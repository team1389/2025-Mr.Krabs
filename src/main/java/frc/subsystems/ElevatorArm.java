package frc.subsystems;

import java.util.HashMap;
import java.util.Map;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;

public class ElevatorArm extends SubsystemBase{
    private SparkFlex elevatorMotorRight, leftShoulderMotor, rightShoulderMotor, elevatorMotorLeft;
    private SparkMax wristMotor;
    double elevatorSpeed = 1;

    boolean shoulderClose = false;
    private RelativeEncoder shoulderRelEncoder, rightElevatorRelEncoder, wristRelEncoder;
    private SparkAbsoluteEncoder wristAbsEncoder;
    public static final SparkFlexConfig elevatorConfig = new SparkFlexConfig();
    public static final SparkFlexConfig shoulderConfig = new SparkFlexConfig();
    public static final SparkMaxConfig wristConfig = new SparkMaxConfig();
    private final SparkClosedLoopController elevatorClosedLoopController;
    private final SparkClosedLoopController shoulderClosedLoopController;
    private final SparkClosedLoopController wristClosedLoopController;

    private ProfiledPIDController elevatorPid = new ProfiledPIDController(RobotMap.ArmConstants.ElevatorP, 
                                                                            RobotMap.ArmConstants.ElevatorI, 
                                                                            RobotMap.ArmConstants.ElevatorD, 
                                                                            new Constraints(RobotMap.ArmConstants.ElevatorMaxVelocity, 
                                                                                            RobotMap.ArmConstants.ElevatorMaxAccerlation));

    private final TrapezoidProfile.Constraints arm1Constraints = new TrapezoidProfile.Constraints(.5, .3);
    // private ProfiledPIDController shoulderPid = new ProfiledPIDController(.5, 0, 0, arm1Constraints);
    private ProfiledPIDController shoulderPid = new ProfiledPIDController(0, 0, 0, arm1Constraints);

    // private PIDController shoulderPid = new PIDController(2, 2, 0);

    private final TrapezoidProfile.Constraints wristConstraints = new TrapezoidProfile.Constraints(3000, 4000);
    private ProfiledPIDController wristPid = new ProfiledPIDController(.03, 0, 0, wristConstraints);
    // private PIDController wristPid = new PIDController(.003, 0, 0);

    private final ElevatorFeedforward elevatorFF = new ElevatorFeedforward(0.02, .9, 3.8, .17);
    private final ArmFeedforward shoulderFF = new ArmFeedforward(0,  0, 0); //ks is static friction, might not need it
    private final ArmFeedforward wristFF = new ArmFeedforward(0, 1.75, 1.95, 0); 

    private double elevatorTarget, shoulderTarget, wristTarget;

    public enum ArmPosition {
        Starting,
        L1,
        L2,
        L3,
        L4,
        Feeder,
        Net
    }

    private final Map<ArmPosition, double[]> positionMap = new HashMap<>();

    public ElevatorArm(){
        elevatorMotorLeft = new SparkFlex(RobotMap.MotorPorts.ELEVATOR_MOTOR_ONE, MotorType.kBrushless);
        elevatorMotorRight = new SparkFlex(RobotMap.MotorPorts.ELEVATOR_MOTOR_TWO, MotorType.kBrushless);
        leftShoulderMotor = new SparkFlex(RobotMap.MotorPorts.LEFT_SHOULDER_MOTOR, MotorType.kBrushless);
        rightShoulderMotor = new SparkFlex(RobotMap.MotorPorts.RIGHT_SHOULDER_MOTOR, MotorType.kBrushless);
        wristMotor = new SparkMax(RobotMap.MotorPorts.WRIST_MOTOR, MotorType.kBrushless);

        shoulderRelEncoder = leftShoulderMotor.getEncoder();
        rightElevatorRelEncoder = elevatorMotorRight.getEncoder();
        wristAbsEncoder = wristMotor.getAbsoluteEncoder();
        wristRelEncoder = wristMotor.getEncoder();

        // wristRelEncoder.setPosition(0);
        wristRelEncoder.setPosition(wristAbsEncoder.getPosition() * 2*Math.PI* RobotMap.ArmConstants.WristGearRatio);

        elevatorClosedLoopController = elevatorMotorRight.getClosedLoopController();
        shoulderClosedLoopController = leftShoulderMotor.getClosedLoopController();
        wristClosedLoopController = wristMotor.getClosedLoopController();

        positionMap.put(ArmPosition.Starting, new double[] {0.5177, .208, .2018});
        positionMap.put(ArmPosition.L1, new double[] {0, 0, 0});
        positionMap.put(ArmPosition.L2, new double[] {32.9788, .03976, .48674});
        positionMap.put(ArmPosition.L3, new double[] {71.4531, .0057, .48773});
        positionMap.put(ArmPosition.L4, new double[] {117.5555, -12.162, .8819});
        positionMap.put(ArmPosition.Feeder, new double[] {0.5177, .19503, .24577});
        positionMap.put(ArmPosition.Net, new double[] {0,0,0});

        elevatorConfig.idleMode(IdleMode.kBrake).smartCurrentLimit(40).voltageCompensation(12);
        elevatorConfig
            .closedLoop
            .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
            // Set PID values for position control
            .p(3)
            .d(.25)
            .outputRange(-1, 1)
            .maxMotion
            // Set MAXMotion parameters for position control
            .maxVelocity(6000) //prev 5000
            .maxAcceleration(10000) //prev 3000, 7000 works
            .allowedClosedLoopError(0.5);

    elevatorMotorRight.configure(elevatorConfig, SparkBase.ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);


        shoulderConfig.idleMode(IdleMode.kBrake).smartCurrentLimit(30).voltageCompensation(12);
            shoulderConfig
                .closedLoop
                .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                // Set PID values for position control
                .p(.5) //prev .5
                .outputRange(-1, 1) //try FF without maxMotion
                // .velocityFF(.5)
                .maxMotion
                // Set MAXMotion parameters for position control
                .maxVelocity(5000) //2000
                .maxAcceleration(100000) //10000
                .allowedClosedLoopError(0.1);

        leftShoulderMotor.configure(shoulderConfig, SparkBase.ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        wristConfig.idleMode(IdleMode.kBrake).smartCurrentLimit(20).voltageCompensation(12);
            wristConfig
                .closedLoop
                .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                .p(.5) //.5
                .outputRange(-1, 1)
                .maxMotion
                .maxVelocity(2000)
                .maxAcceleration(10000)
                .allowedClosedLoopError(1);

            // wristConfig
            //     .softLimit
            //         .forwardSoftLimit(.1)
            //         .forwardSoftLimitEnabled(true)
            //         .reverseSoftLimit(.6)
            //         .reverseSoftLimitEnabled(true);

        wristMotor.configure(wristConfig, SparkBase.ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);


    }

    public void zeroElevator(){
        rightElevatorRelEncoder.setPosition(0);
    }


    public void setManualElevator(double elevatorSpeed){
        // elevatorMotorLeft.set(elevatorSpeed);
        elevatorMotorRight.set(-elevatorSpeed);
    }
    public void setManualShoulder(double arm1Speed){
        leftShoulderMotor.set(arm1Speed);
        // rightShoulderMotor.set(-arm1Speed);
    }
    public void setManualWrist(double arm2Speed){
        // MathUtil.clamp(arm2Speed, -0.3, 0.3);
        // if (getWristEncoder() < 0.025 && arm2Speed > 0){
        //     wristMotor.set(0);
        // }
        // else if (getWristEncoder() > 0.975 && arm2Speed < 0){
        //     wristMotor.set(0);
        // }
        // else{
        //     wristMotor.set(arm2Speed);
        // }
        wristMotor.set(arm2Speed);
    }


    public double getRightRelElevatorPos(){
        return rightElevatorRelEncoder.getPosition();
    }

    public double getShoulderRelPos(){
        return shoulderRelEncoder.getPosition();
    }

    public double getWristPos(){
        return wristAbsEncoder.getPosition();
    }

    public double getWristRelPos(){
        return wristRelEncoder.getPosition();
    }

    public void setElevatorArm(ArmPosition pos){
        double[] targets = positionMap.get(pos);
        elevatorTarget = targets[0];
        shoulderTarget = targets[1];
        wristTarget = targets[2];
    }

    public void setElevator(double goal){

        // elevatorPid.setGoal(goal);
        // elevatorPid.setTolerance(.001);
        double speed = ((elevatorPid.calculate(getRightRelElevatorPos(), goal)));
        double FF = elevatorFF.calculate(elevatorPid.getSetpoint().velocity);
        // elevatorMotorRight.setVoltage(MathUtil.clamp(speed+FF, -7, 7));
        elevatorMotorRight.set(speed+FF);
        SmartDashboard.putNumber("Elevator Goal", goal);
    }

    public void moveToSetpoint(double goal) {
    elevatorClosedLoopController.setReference(
        goal, ControlType.kPosition);
        SmartDashboard.putBoolean("in closedLoop Elevator", true);
    }

    public void reachGoal(double goal){
        double voltsOutput = MathUtil.clamp(
            elevatorFF.calculateWithVelocities(getVelocityMetersPerSecond(), elevatorPid.getSetpoint().velocity)
            + elevatorPid.calculate(getPositionMeters(), goal), 
            -7, 
            7);
            elevatorMotorRight.setVoltage(voltsOutput);
    }

    public void moveToSetpointShoulder(double goal){
        shoulderClosedLoopController.setReference(
        goal, ControlType.kPosition);
    }

    public void setShoulder(double setpoint){
        // shoulderPid.setTolerance(0.001);
        double speed = ((shoulderPid.calculate(getShoulderRelPos(), setpoint)));
        double FF = shoulderFF.calculate(shoulderPid.getSetpoint().position, shoulderPid.getSetpoint().velocity);
        
        // setManualShoulder(MathUtil.clamp(speed + FF, -.4, .4));
        setManualShoulder(speed + FF);
    }

    public void moveToSetpointWrist(double goal){
        wristClosedLoopController.setReference(goal, ControlType.kMAXMotionPositionControl);
    }

    public void setWrist(double setpoint){
        // double currentAngle = getWristEncoder();
        // double power = wristPid.calculate(currentAngle, setpoint);

        // Limit the power to prevent damage to the mechanism.  These values
        // should be based on your hardware.
        // power = -MathUtil.clamp(power, -.3, .3); // Example: Limit between -1 and 1
        double power = -wristPid.calculate(getWristRelPos(), setpoint);
        wristMotor.set(MathUtil.clamp(power, -.4, .4));
    }


    public void stop(){
        elevatorMotorRight.set(0);
        leftShoulderMotor.set(0);
        wristMotor.set(0);
    }

    public boolean atTargetPosition(){
        boolean elevatorClose = atTargetPosition(elevatorTarget);
        boolean shoulderClose = atShoulderTargetPosition(shoulderTarget);
        boolean wristClose = atWristTargetPosition(wristTarget);
    
        return elevatorClose && shoulderClose && wristClose;
    }

    public boolean atTargetPosition(double height){
        boolean elevatorClose = Math.abs(getRightRelElevatorPos() - height) < .5;
        SmartDashboard.putBoolean("Elevator At Target", elevatorClose);
        return elevatorClose;
    }

    public boolean  atWristTargetPosition(double height){
        boolean wristClose = Math.abs(getWristRelPos() - height) < 1;
        SmartDashboard.putBoolean("Wrist At Target", wristClose);
        return wristClose;
    }

    public boolean atShoulderTargetPosition(double height){
        shoulderClose = Math.abs(getShoulderRelPos() - height) < .05;
        SmartDashboard.putBoolean("Shoulder At Target", shoulderClose);
        return shoulderClose;
    }

    public void moveElevatorArm(){
        setElevator(elevatorTarget);
        moveToSetpointShoulder(shoulderTarget);
        setWrist(wristTarget);
    }

    public double getVelocityMetersPerSecond() {
        return (rightElevatorRelEncoder.getVelocity() /60) * (2*Math.PI * RobotMap.ArmConstants.ElevatorDrumRadius)
        / RobotMap.ArmConstants.ElevatorGearing;
    }

    public double getPositionMeters(){
        return rightElevatorRelEncoder.getPosition() * (2*Math.PI * RobotMap.ArmConstants.ElevatorDrumRadius)
        / RobotMap.ArmConstants.ElevatorGearing;
    }

    @Override
    public void periodic(){
        wristRelEncoder.setPosition(wristAbsEncoder.getPosition() * 2*Math.PI* RobotMap.ArmConstants.WristGearRatio);
        // SmartDashboard.putNumber("wrist acc error", wristPid.getAccumulatedError());
        // SmartDashboard.putNumber("wrist error", wristPid.getPositionError());

        // SmartDashboard.putNumber("elevator M per S", getVelocityMetersPerSecond());
        // SmartDashboard.putNumber("elevator position M", getPositionMeters());

        // SmartDashboard.putBoolean("Shoulder At Target", shoulderClose);

        // elevatorPid.setP(SmartDashboard.getNumber("P Elevator", 0));
        // elevatorPid.setP(SmartDashboard.getNumber("I Elevator", 0));
        // elevatorPid.setP(SmartDashboard.getNumber("D Elevator", 0));

        SmartDashboard.putNumber("Right Elevator Pos", getRightRelElevatorPos());
        SmartDashboard.putNumber("Shoulder Rel Pos", getShoulderRelPos());
        SmartDashboard.putNumber("Wrist Abs Position", getWristPos());
        SmartDashboard.putNumber("Wrist Rel Pos", getWristRelPos());

        // SmartDashboard.putNumber("Elevator error", elevatorPid.getAccumulatedError());
        // SmartDashboard.putNumber("Elevator Position error", elevatorPid.getPositionError());

        // SmartDashboard.putNumber("P Elevator", 0);
        // SmartDashboard.putNumber("I Elevator", 0);
        // SmartDashboard.putNumber("D Elevator", 0);
        // SmartDashboard.putNumber("K Elevator", 0);
        // SmartDashboard.putNumber("G Elevator", 0);
        // SmartDashboard.putNumber("A Elevator", 0);
     //   SmartDashboard.putNumber("Arm1 FF", shoulderFF.get());
    }
}
