package frc.subsystems;

import java.util.HashMap;
import java.util.Map;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.servohub.ServoHub.ResetMode;
import com.revrobotics.sim.SparkFlexSim;
import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.simulation.DIOSim;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.RobotMap;

public class ElevatorArm extends SubsystemBase{
    private SparkFlex elevatorMotorRight, leftShoulderMotor, rightShoulderMotor, elevatorMotorLeft;
    private SparkMax wristMotor; //.12 to .85
    // shoulder is a spark max
    double elevatorSpeed = 1;   

    double pS,iS,dS;
    boolean shoulderClose = false;
    private final DCMotor elevatorGearbox = DCMotor.getNeoVortex(1);
    private SparkFlexSim elevatorMotorSim;
    private RelativeEncoder shoulderRelEncoder, rightElevatorRelEncoder; //-.3 to -110
    // private AbsoluteEncoder wristAbsEncoder;
    // private DutyCycleEncoder wristEncoder;
    public static final SparkFlexConfig elevatorConfig = new SparkFlexConfig();
    public static final SparkFlexConfig shoulderConfig = new SparkFlexConfig();
    private final SparkClosedLoopController elevatorClosedLoopController;
    private final SparkClosedLoopController shoulderClosedLoopController;

    // private DigitalInput topLimitSwitch, bottomLimitSwitch;

    // private final TrapezoidProfile.Constraints elevatorConstraints = new TrapezoidProfile.Constraints(50, 30); //TODO
    private ProfiledPIDController elevatorPid = new ProfiledPIDController(RobotMap.ArmConstants.ElevatorP, 
                                                                            RobotMap.ArmConstants.ElevatorI, 
                                                                            RobotMap.ArmConstants.ElevatorD, 
                                                                            new Constraints(RobotMap.ArmConstants.ElevatorMaxVelocity, 
                                                                                            RobotMap.ArmConstants.ElevatorMaxAccerlation));
    // private PIDController elevatorPid = new PIDController(.25, 0, 0);

    private final TrapezoidProfile.Constraints arm1Constraints = new TrapezoidProfile.Constraints(.5, .3); //TODO
    // private ProfiledPIDController shoulderPid = new ProfiledPIDController(.5, 0, 0, arm1Constraints);
    private ProfiledPIDController shoulderPid = new ProfiledPIDController(0, 0, 0, arm1Constraints);

    // private PIDController shoulderPid = new PIDController(2, 2, 0);

    private final TrapezoidProfile.Constraints wristConstraints = new TrapezoidProfile.Constraints(3, 3); //TODO
    // private ProfiledPIDController wristPid = new ProfiledPIDController(.003, 0, 1, wristConstraints);
    private PIDController wristPid = new PIDController(2, 0, 0);

    //TODO
    // private final ElevatorFeedforward elevatorFF = new ElevatorFeedforward(0, 2.28, 3.07, .41);
    private final ElevatorFeedforward elevatorFF = new ElevatorFeedforward(0.02, .9, 3.8, .17);
    private final ArmFeedforward shoulderFF = new ArmFeedforward(0,  0, 0); //ks is static friction, might not need it
    private final ArmFeedforward wristFF = new ArmFeedforward(0, 1.75, 1.95, 0); 

    private SparkAbsoluteEncoder wristAbsEncoder;

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

    private ElevatorSim elevatorSim = null;
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
        // elevatorEncoder = new DutyCycleEncoder(0, Math.PI, 0); // 0 to PI
        // wristEncoder = new DutyCycleEncoder(1, Math.PI, 0);
 
        //limit switch, classic JJ
        // topLimitSwitch = new DigitalInput(RobotMap.ArmConstants.TOP_LIMIT_SWITCH);
        // bottomLimitSwitch = new DigitalInput(RobotMap.ArmConstants.BOTTOM_LIMIT_SWITCH);
        elevatorClosedLoopController = elevatorMotorRight.getClosedLoopController();
        shoulderClosedLoopController = leftShoulderMotor.getClosedLoopController();

        positionMap.put(ArmPosition.Starting, new double[] {20, 0, .65});
        positionMap.put(ArmPosition.L1, new double[] {0,0,0});
        positionMap.put(ArmPosition.L2, new double[] {0,0,0});
        positionMap.put(ArmPosition.L3, new double[] {0,0,0});
        positionMap.put(ArmPosition.L4, new double[] {0, 0, 0});
        positionMap.put(ArmPosition.Feeder, new double[] {0,0,0});
        positionMap.put(ArmPosition.Net, new double[] {0,0,0});

        // setInverted(rightShoulderMotor);
       // setInverted(elevatorMotorLeft);
        // zeroShoulderRelEncoder();
        // setElevatorArm(ArmPosition.Starting);

            elevatorConfig.idleMode(IdleMode.kBrake).smartCurrentLimit(40).voltageCompensation(12);
            elevatorConfig
                .closedLoop
                .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                // Set PID values for position control
                .p(3)
                .outputRange(-1, 1)
                .maxMotion
                // Set MAXMotion parameters for position control
                .maxVelocity(42000)
                .maxAcceleration(60000)
                .allowedClosedLoopError(0.5);

        elevatorMotorRight.configure(elevatorConfig, SparkBase.ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        shoulderConfig.idleMode(IdleMode.kBrake).smartCurrentLimit(30).voltageCompensation(12);
            shoulderConfig
                .closedLoop
                .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                // Set PID values for position control
                .p(.5)
                .outputRange(-1, 1)
                .maxMotion
                // Set MAXMotion parameters for position control
                .maxVelocity(2000)
                .maxAcceleration(10000)
                .allowedClosedLoopError(0.05);

        leftShoulderMotor.configure(shoulderConfig, SparkBase.ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        // SparkFlexConfig config = new SparkFlexConfig();
        // config.smartCurrentLimit(40)
        //         .openLoopRampRate(RobotMap.ArmConstants.ElevatorRampRate);

        wristAbsEncoder = wristMotor.getAbsoluteEncoder();

        wristPid.setTolerance(0.005);

        // SmartDashboard.putNumber("P Elevator", 0);
        // SmartDashboard.putNumber("I Elevator", 0);
        // SmartDashboard.putNumber("D Elevator", 0);
        // SmartDashboard.putNumber("K Elevator", 0);
        // SmartDashboard.putNumber("G Elevator", 0);
        // SmartDashboard.putNumber("A Elevator", 0);

        if (RobotBase.isSimulation()) {
            elevatorSim = new ElevatorSim(elevatorGearbox,
                    RobotMap.ArmConstants.ElevatorGearing,
                    RobotMap.ArmConstants.ElevatorCarriageMass,
                    RobotMap.ArmConstants.ElevatorDrumRadius,
                    RobotMap.ArmConstants.ElevatorMinHeight,
                    RobotMap.ArmConstants.ElevatorMaxHeight,
                    true,
                    0.0,
                    0.02,
                    0.0);
        }

    }
    // public void setInverted(SparkFlex motor){
    //     configs.inverted(true);
    // }

    // public void updateMotorSettings(SparkFlex motor){
    //     configs.IdleMode(IdleMode.kBrake);
    // }

        //    public double getRadians(){
        //     double radians=encoder.getRPM*2*Math.PI
        //     radians/=60;

        // }

        // public void holdPosition(){
            
        // }


    public void setManualElevator(double elevatorSpeed){
        // elevatorMotorLeft.set(elevatorSpeed);
        elevatorMotorRight.set(-elevatorSpeed);
    }
    public void setManualElevatorUp(){
        // elevatorMotorLeft.set(elevatorSpeed);
        elevatorMotorRight.set(-elevatorSpeed);
    }
    public void setManualElevatorDown(){
        // elevatorMotorLeft.set(-elevatorSpeed);
        elevatorMotorRight.set(elevatorSpeed);
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

    public double getWristEncoder(){
        return wristAbsEncoder.getPosition();
    }


    // public void setSpeed(SparkFlex motor, double speed, double maxSpeed){
    //     double setSpeed = Math.max(-maxSpeed, Math.min(speed, maxSpeed));
    //     motor.set(setSpeed);
    // }


    public double getRightRelElevatorPos(){
        return rightElevatorRelEncoder.getPosition();
    }
    // public double getShoulderPos(){
    //     return shoulderEncoder.get();
    // }
    public double getShoulderRelPos(){
        return shoulderRelEncoder.getPosition();
    }

    public double getWristPos(){
        return wristAbsEncoder.getPosition();
    }

    public void setElevatorArm(ArmPosition pos){
        double[] targets = positionMap.get(pos);
        elevatorTarget = targets[0];
        shoulderTarget = targets[1];
        wristTarget = targets[2];
    }

    public void setElevator(double goal){
        // if(ifElevatorTooLow()){
        //     SmartDashboard.putBoolean("in set elevator low", true);
        //     return;
        // }
        // SmartDashboard.putBoolean("in set elevator low", false);

        // elevatorPid.setGoal(goal);
        // elevatorPid.setTolerance(.001);
        double speed = ((elevatorPid.calculate(getRightRelElevatorPos(), goal)));
        double FF = elevatorFF.calculate(elevatorPid.getSetpoint().velocity);
        // elevatorMotorRight.setVoltage(MathUtil.clamp(speed);
        elevatorMotorRight.set(speed+FF);
        // setManualElevator((speed + FF));
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
        goal, ControlType.kMAXMotionPositionControl);
    }

    public void setShoulder(double setpoint){
        // if(ifShoulderTooLow()){
        //     return;
        // }
        // shoulderPid.setTolerance(0.001);
        double speed = ((shoulderPid.calculate(getShoulderRelPos(), setpoint)));
        double FF = shoulderFF.calculate(shoulderPid.getSetpoint().position, shoulderPid.getSetpoint().velocity);
        
        // setManualShoulder(MathUtil.clamp(speed + FF, -.4, .4));
        setManualShoulder(speed + FF);
    }

    public void setWrist(double setpoint){
        double currentAngle = getWristEncoder();
        double power = wristPid.calculate(currentAngle, setpoint);

        // Limit the power to prevent damage to the mechanism.  These values
        // should be based on your hardware.
        power = -MathUtil.clamp(power, -.3, .3); // Example: Limit between -1 and 1

        setManualWrist(power);
        // if(ifElevatorTooLow()){
        //     SmartDashboard.putBoolean("in set elevator low", true);
        //     return;
        // }
        // SmartDashboard.putBoolean("in set wrist low", false);
        // double speed = wristPid.calculate(getWristPos(), setpoint);
        // setManualWrist(MathUtil.clamp(speed, -.3, .3));
    }


    public void stop(){
        // elevatorMotorLeft.set(0);
        elevatorMotorRight.set(0);
        leftShoulderMotor.set(0);
        wristMotor.set(0);
    }

    public boolean ifElevatorTooHigh(){
        return rightElevatorRelEncoder.getPosition() > 100;
    }

    public boolean ifElevatorTooLow(){
        return rightElevatorRelEncoder.getPosition() < 0;
    }

    //what is this bro
    public boolean ifShoulderTooLow(){
        return rightElevatorRelEncoder.getPosition() < 0;
    }

    // public boolean ifWristTooFar(){
    //     return rightElevatorRelEncoder.getPosition() > 100; //TODO
    // }ma

    // public boolean ifWristTooLow(){
    //     return rightElevatorRelEncoder.getPosition() < 0; //TODO
    // }

    // public void zeroShoulderRelEncoder(){
    //     while(!topLimitSwitch.get()){
    //         leftShoulderMotor.set(.3);
    //         rightShoulderMotor.set(-.3);
    //     }
    //     leftShoulderMotor.set(0);
    //     rightShoulderMotor.set(0);
    //     shoulderRelEncoder.setPosition(0);
    // }

    public void moveElevator(double power){
        // elevatorMotorLeft.setVoltage(MathUtil.clamp(power, -12, 12));
        elevatorMotorRight.setVoltage(-MathUtil.clamp(power, -12, 12));
    }
    public void moveShoulder(double power){ 
        // if((shoulderTarget > 90 || shoulderTarget < 0) && power > 0){ //TODO
        //     leftShoulderMotor.setVoltage(0);
        //     rightShoulderMotor.setVoltage(0);
        //     SmartDashboard.putBoolean("Shoulder: Too High or Too Low", true);
        //     return;
        // }
        leftShoulderMotor.setVoltage(MathUtil.clamp(power, -12, 12));
        // rightShoulderMotor.setVoltage(MathUtil.clamp(power, -12, 12));
        // SmartDashboard.putBoolean("Shoulder: Too High or Too Low", false);
    }
    public void moveWrist(double power){
        // if((wristTarget > 90 || wristTarget < 0) && power > 0){ //TODO
        //     wristMotor.setVoltage(0);
        //     SmartDashboard.putBoolean("Wrist: Too High or Too Low", true);
        //     return;
        // }
        wristMotor.setVoltage(MathUtil.clamp(power, -12, 12));
        // SmartDashboard.putBoolean("Shoulder: Too High or Too Low", false);
    }

    public boolean atTargetPosition(){
        boolean elevatorClose = Math.abs(getRightRelElevatorPos() - elevatorTarget) < 0.5;
    //  boolean shoulderClose = Math.abs(getShoulderPos() - shoulderTarget) < 0.05;
        boolean shoulderClose = Math.abs(getShoulderRelPos() - shoulderTarget) < 0.05;
        boolean wristClose = Math.abs(getWristPos() - wristTarget) < 0.005;
    
        SmartDashboard.putBoolean("Elevator Target", elevatorClose);
        SmartDashboard.putBoolean("Shoulder Target", shoulderClose);
        SmartDashboard.putBoolean("Wrist Target", wristClose);
    
        return elevatorClose && shoulderClose && wristClose;
    }

    public boolean atTargetPosition(double height){
        boolean elevatorClose = Math.abs(getRightRelElevatorPos() - height) < .5;
        SmartDashboard.putBoolean("Elevator At Target", elevatorClose);
        return elevatorClose;
    }

    public boolean atWristTargetPosition(double height){
        boolean wristClose = Math.abs(getWristPos() - height) < .1;
        SmartDashboard.putBoolean("Wrist At Target", wristClose);
        return wristClose;
    }

    public boolean atShoulderTargetPosition(double height){
        shoulderClose = Math.abs(getShoulderRelPos() - height) < .05;
        // SmartDashboard.putBoolean("Shoulder At Target", shoulderClose);
        return shoulderClose;
    }

    public void moveElevatorArm(){
        double elevatorPower = elevatorPid.calculate(getRightRelElevatorPos(), elevatorTarget);
        double shoulderPower = shoulderPid.calculate(getShoulderRelPos(), shoulderTarget);// + shoulderFF.calculate(shoulderTarget, 0); // for limit switch
        double wristPower = wristPid.calculate(getWristPos(), wristTarget);

        moveElevator(elevatorPower);
        moveShoulder(shoulderPower);
        moveWrist(wristPower);
    }

    // public void setElevator(double height){
    //     height = MathUtil.clamp(height, 0, 110);
    //     double power = elevatorPid.calculate(getRightRelElevatorPos(), height);
    //     moveElevator(power);
    // }

    public void setRadians(){

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
        SmartDashboard.putNumber("elevator M/S", getVelocityMetersPerSecond());
        SmartDashboard.putNumber("elevator position M", getPositionMeters());

        SmartDashboard.putBoolean("Shoulder At Target", shoulderClose);
        // double elevatorPower = elevatorPid.calculate(getRightRelElevatorPos(), elevatorTarget);
        // double shoulderPower = shoulderPid.calculate(getShoulderRelPos(), shoulderTarget);// + shoulderFF.calculate(shoulderTarget, 0); // for limit switch
        // double wristPower = wristPid.calculate(getWristPos(), wristTarget);// + wristFF.calculate(wristTarget, 0); // add FF TODO

        // moveElevator(elevatorPower);
        // moveShoulder(shoulderPower);
        // moveWrist(wristPower);

        // if(topLimitSwitch.get()){
        //     shoulderRelEncoder.setPosition(0);
        // }

        // -.3 to -110

        // if(getRightRelElevatorPos() < 0 || getRightRelElevatorPos() > 50){
        //     stop();
        // }

        // shoulderPid.setPID(SmartDashboard.getNumber("P Shoulder", 5), 
        // SmartDashboard.getNumber("I Shoulder", 0), 
        // SmartDashboard.getNumber("D Shoulder", 1));

        shoulderPid.setP(SmartDashboard.getNumber("P Shoulder", 4));
        shoulderPid.setI(SmartDashboard.getNumber("I Shoulder", 0));
        shoulderPid.setD(SmartDashboard.getNumber("D Shoulder", 0));

        // elevatorPid.setP(SmartDashboard.getNumber("P Elevator", 0));
        // elevatorPid.setP(SmartDashboard.getNumber("I Elevator", 0));
        // elevatorPid.setP(SmartDashboard.getNumber("D Elevator", 0));

        

        // SmartDashboard.putNumber("Left Elevator Pos", getLeftRelElevatorPos());
        SmartDashboard.putNumber("Right Elevator Pos", getRightRelElevatorPos());
    // //    SmartDashboard.putNumber("Shoulder Position", getShoulderPos());
        SmartDashboard.putNumber("Shoulder Rel Pos", getShoulderRelPos());
        SmartDashboard.putNumber("Wrist Position", getWristPos());

        SmartDashboard.putNumber("Elevator error", elevatorPid.getAccumulatedError());
        SmartDashboard.putNumber("Elevator Position error", elevatorPid.getPositionError());

        // SmartDashboard.putNumber("P Elevator", 0);
        // SmartDashboard.putNumber("I Elevator", 0);
        // SmartDashboard.putNumber("D Elevator", 0);
        // SmartDashboard.putNumber("K Elevator", 0);
        // SmartDashboard.putNumber("G Elevator", 0);
        // SmartDashboard.putNumber("A Elevator", 0);
     //   SmartDashboard.putNumber("Arm1 FF", shoulderFF.get());
    }
}
