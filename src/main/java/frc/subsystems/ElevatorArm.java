package frc.subsystems;

import java.util.HashMap;
import java.util.Map;

import com.revrobotics.*;
import com.revrobotics.spark.*;
import com.revrobotics.spark.config.*;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.*;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;

/**
 * ElevatorArm controls the robot’s arm system, which includes:
 * - An elevator (vertical movement)
 * - A shoulder joint (pivot)
 * - A wrist joint (rotation at the end of the arm)
 *
 * It uses PID + feedforward control for precise positioning,
 * and includes simulation support for testing.
 */
public class ElevatorArm extends SubsystemBase {

    // Motor controllers for each part of the arm
    private SparkFlex elevatorMotorRight, elevatorMotorLeft, leftShoulderMotor, rightShoulderMotor;
    private SparkMax wristMotor;

    // Encoders for reading motor positions
    private RelativeEncoder shoulderRelEncoder, rightElevatorRelEncoder, wristRelEncoder;
    private SparkAbsoluteEncoder wristAbsEncoder;

    // Feedforward & PID controllers for each axis
    private final SparkClosedLoopController elevatorClosedLoopController;
    private final SparkClosedLoopController shoulderClosedLoopController;
    private final SparkClosedLoopController wristClosedLoopController;

    // PID controllers with motion profiling for smoother movements
    private ProfiledPIDController elevatorPid, shoulderPid, wristPid;

    // Feedforward terms to compensate for gravity/friction
    private final ElevatorFeedforward elevatorFF;
    private final ArmFeedforward shoulderFF, wristFF;

    // Motion configuration profiles
    public static final SparkFlexConfig elevatorConfig = new SparkFlexConfig();
    public static final SparkFlexConfig shoulderConfig = new SparkFlexConfig();
    public static final SparkMaxConfig wristConfig = new SparkMaxConfig();

    // Simulation objects for testing without hardware
    private final DCMotor elevatorGearbox = DCMotor.getNeoVortex(1);
    private ElevatorSim elevatorSim = null;

    // Default speed for manual control
    double elevatorSpeed = 1;

    // Target positions (for automatic control)
    public double elevatorTarget, shoulderTarget, wristTarget;

    // Used to track whether shoulder is near its setpoint
    boolean shoulderClose = false;

    // Enum to represent preset arm positions
    public enum ArmPosition {
        Starting,
        L1, L2, L3, L4,
        Feeder,
        Net
    }

    // Stores preset target positions for elevator, shoulder, and wrist
    private final Map<ArmPosition, double[]> positionMap = new HashMap<>();

    /**
     * Constructor initializes motors, encoders, control loops, and configurations.
     */
    public ElevatorArm() {
        // Initialize motors
        elevatorMotorLeft = new SparkFlex(RobotMap.MotorPorts.ELEVATOR_MOTOR_ONE, MotorType.kBrushless);
        elevatorMotorRight = new SparkFlex(RobotMap.MotorPorts.ELEVATOR_MOTOR_TWO, MotorType.kBrushless);
        leftShoulderMotor = new SparkFlex(RobotMap.MotorPorts.LEFT_SHOULDER_MOTOR, MotorType.kBrushless);
        rightShoulderMotor = new SparkFlex(RobotMap.MotorPorts.RIGHT_SHOULDER_MOTOR, MotorType.kBrushless);
        wristMotor = new SparkMax(RobotMap.MotorPorts.WRIST_MOTOR, MotorType.kBrushless);

        // Get encoders for position/velocity feedback
        shoulderRelEncoder = leftShoulderMotor.getEncoder();
        rightElevatorRelEncoder = elevatorMotorRight.getEncoder();
        wristAbsEncoder = wristMotor.getAbsoluteEncoder();
        wristRelEncoder = wristMotor.getEncoder();

        // Sync relative wrist encoder to absolute encoder value
        wristRelEncoder.setPosition(wristAbsEncoder.getPosition() * 2 * Math.PI * RobotMap.ArmConstants.WristGearRatio);

        // Get closed-loop control interfaces
        elevatorClosedLoopController = elevatorMotorRight.getClosedLoopController();
        shoulderClosedLoopController = leftShoulderMotor.getClosedLoopController();
        wristClosedLoopController = wristMotor.getClosedLoopController();

        // Define preset positions for autonomous control
        positionMap.put(ArmPosition.Starting, new double[] {0.5177, .208, .2018});
        positionMap.put(ArmPosition.L1, new double[] {0, 0, 0});
        positionMap.put(ArmPosition.L2, new double[] {32.9788, .03976, .48674});
        positionMap.put(ArmPosition.L3, new double[] {71.4531, .0057, .48773});
        positionMap.put(ArmPosition.L4, new double[] {117.5555, -12.162, 265});
        positionMap.put(ArmPosition.Feeder, new double[] {0.5177, .19503, .24577});
        positionMap.put(ArmPosition.Net, new double[] {0,0,0});

        // Create PID controllers with motion constraints
        elevatorPid = new ProfiledPIDController(
            RobotMap.ArmConstants.ElevatorP,
            RobotMap.ArmConstants.ElevatorI,
            RobotMap.ArmConstants.ElevatorD,
            new Constraints(
                RobotMap.ArmConstants.ElevatorMaxVelocity,
                RobotMap.ArmConstants.ElevatorMaxAccerlation
            )
        );

        shoulderPid = new ProfiledPIDController(0, 0, 0, new Constraints(.5, .3));
        wristPid = new ProfiledPIDController(.03, 0, 0, new Constraints(3000, 4000));

        // Feedforward controllers (gravity/friction compensation)
        elevatorFF = new ElevatorFeedforward(0.02, .9, 3.8, .17);
        shoulderFF = new ArmFeedforward(0, 0, 0);
        wristFF = new ArmFeedforward(0, 1.75, 1.95, 0);

        // Configure motor behavior
        configureMotors();

        // Initialize simulation if running in simulator
        if (RobotBase.isSimulation()) {
            elevatorSim = new ElevatorSim(
                elevatorGearbox,
                RobotMap.ArmConstants.ElevatorGearing,
                RobotMap.ArmConstants.ElevatorCarriageMass,
                RobotMap.ArmConstants.ElevatorDrumRadius,
                RobotMap.ArmConstants.ElevatorMinHeight,
                RobotMap.ArmConstants.ElevatorMaxHeight,
                true,
                0.0, 0.02, 0.0);
        }
    }

    /**
     * Configure the motor control settings, including PID and current limits.
     */
    private void configureMotors() {
        // Elevator motor configuration
        elevatorConfig.idleMode(IdleMode.kBrake)
                      .smartCurrentLimit(40)
                      .voltageCompensation(12);
        elevatorConfig.closedLoop
                      .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                      .p(3).d(.25)
                      .outputRange(-1, 1)
                      .maxMotion.maxVelocity(5000).maxAcceleration(3000)
                      .allowedClosedLoopError(0.5);
        elevatorMotorRight.configure(elevatorConfig, SparkBase.ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        // Shoulder configuration
        shoulderConfig.idleMode(IdleMode.kBrake)
                      .smartCurrentLimit(30)
                      .voltageCompensation(12);
        shoulderConfig.closedLoop
                      .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                      .p(.5)
                      .outputRange(-1, 1)
                      .maxMotion.maxVelocity(5000)
                      .maxAcceleration(10000)
                      .allowedClosedLoopError(0.05);
        leftShoulderMotor.configure(shoulderConfig, SparkBase.ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    /* -------------------- Manual Control Methods -------------------- */

    // Move elevator up/down manually (for tuning or override)
    public void setManualElevator(double speed) {
        elevatorMotorRight.set(-speed);
    }

    // Manually move shoulder joint
    public void setManualShoulder(double speed) {
        leftShoulderMotor.set(speed);
    }

    // Manually move wrist joint
    public void setManualWrist(double speed) {
        wristMotor.set(speed);
    }

    /* -------------------- Sensor Reading Methods -------------------- */

    public double getRightRelElevatorPos() { return rightElevatorRelEncoder.getPosition(); }
    public double getShoulderRelPos() { return shoulderRelEncoder.getPosition(); }
    public double getWristPos() { return wristAbsEncoder.getPosition(); }
    public double getWristRelPos() { return wristRelEncoder.getPosition(); }

    /* -------------------- Target Position Methods -------------------- */

    // Set target positions from preset
    public void setElevatorArm(ArmPosition pos) {
        double[] targets = positionMap.get(pos);
        elevatorTarget = targets[0];
        shoulderTarget = targets[1];
        wristTarget = targets[2];
    }

    // Individual target accessors (with SmartDashboard logging)
    public double returnElevatorTarget(ArmPosition pos) {
        double[] t = positionMap.get(pos);
        elevatorTarget = t[0];
        SmartDashboard.putNumber("Elevator Setpoint", elevatorTarget);
        return elevatorTarget;
    }

    public double returnShoulderTarget(ArmPosition pos) {
        double[] t = positionMap.get(pos);
        shoulderTarget = t[1];
        SmartDashboard.putNumber("Shoulder Setpoint", shoulderTarget);
        return shoulderTarget;
    }

    public double returnWristTarget(ArmPosition pos) {
        double[] t = positionMap.get(pos);
        wristTarget = t[2];
        SmartDashboard.putNumber("Wrist Setpoint", wristTarget);
        return wristTarget;
    }

    /* -------------------- Automated Control Methods -------------------- */

    // PID + feedforward elevator control
    public void setElevator(double goal) {
        double pidOut = elevatorPid.calculate(getRightRelElevatorPos(), goal);
        double ffOut = elevatorFF.calculate(elevatorPid.getSetpoint().velocity);
        elevatorMotorRight.set(pidOut + ffOut);
    }

    // Motion profiling position control for shoulder and wrist
    public void moveToSetpointShoulder(double goal) {
        shoulderClosedLoopController.setReference(goal, ControlType.kMAXMotionPositionControl);
    }

    public void moveToSetpointWrist(double goal) {
        wristClosedLoopController.setReference(goal, ControlType.kMAXMotionPositionControl);
    }

    // Wrist PID control with output limiting
    public void setWrist(double setpoint) {
        double power = -wristPid.calculate(getWristRelPos(), setpoint);
        wristMotor.set(MathUtil.clamp(power, -.3, .3));
    }

    // Stops all motors
    public void stop() {
        elevatorMotorRight.set(0);
        leftShoulderMotor.set(0);
        wristMotor.set(0);
    }

    /* -------------------- Position Checking Methods -------------------- */

    public boolean atTargetPosition() {
        return atTargetPosition(elevatorTarget) &&
               atShoulderTargetPosition(shoulderTarget) &&
               atWristTargetPosition(wristTarget);
    }

    public boolean atTargetPosition(double height) {
        boolean close = Math.abs(getRightRelElevatorPos() - height) < .5;
        SmartDashboard.putBoolean("Elevator At Target", close);
        return close;
    }

    public boolean atWristTargetPosition(double height) {
        boolean close = Math.abs(getWristRelPos() - height) < 1;
        SmartDashboard.putBoolean("Wrist At Target", close);
        return close;
    }

    public boolean atShoulderTargetPosition(double height) {
        shoulderClose = Math.abs(getShoulderRelPos() - height) < .05;
        return shoulderClose;
    }

    /* -------------------- Simulation & Dashboard -------------------- */

    @Override
    public void periodic() {
        // Update wrist position to match absolute encoder each cycle
        wristRelEncoder.setPosition(wristAbsEncoder.getPosition() * 2 * Math.PI * RobotMap.ArmConstants.WristGearRatio);

        // Dashboard feedback for debugging
        SmartDashboard.putNumber("Wrist Rel Pos", getWristRelPos());
        SmartDashboard.putNumber("Right Elevator Pos", getRightRelElevatorPos());
        SmartDashboard.putNumber("Shoulder Rel Pos", getShoulderRelPos());
        SmartDashboard.putNumber("Elevator M/s", getVelocityMetersPerSecond());
        SmartDashboard.putBoolean("Shoulder At Target", shoulderClose);

        // Allow live PID tuning from SmartDashboard
        shoulderPid.setP(SmartDashboard.getNumber("P Shoulder", 4));
        shoulderPid.setI(SmartDashboard.getNumber("I Shoulder", 0));
        shoulderPid.setD(SmartDashboard.getNumber("D Shoulder", 0));
    }

    // Converts encoder velocity to physical units
    public double getVelocityMetersPerSecond() {
        return (rightElevatorRelEncoder.getVelocity() / 60)
                * (2 * Math.PI * RobotMap.ArmConstants.ElevatorDrumRadius)
                / RobotMap.ArmConstants.ElevatorGearing;
    }

    // Converts encoder position to meters
    public double getPositionMeters() {
        return rightElevatorRelEncoder.getPosition()
                * (2 * Math.PI * RobotMap.ArmConstants.ElevatorDrumRadius)
                / RobotMap.ArmConstants.ElevatorGearing;
    }
}
