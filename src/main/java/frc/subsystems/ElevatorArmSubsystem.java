package frc.subsystems;

import java.util.HashMap;
import java.util.Map;

// import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.RelativeEncoder;
// import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
// import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ElevatorFeedforward;
// import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;

public class ElevatorArmSubsystem extends SubsystemBase{
    private SparkFlex elevatorMotorOne, leftShoulderMotor, rightShoulderMotor, wristMotor, elevatorMotorTwo;

    private RelativeEncoder shoulderRelEncoder;
    private DutyCycleEncoder elevatorEncoder, wristEncoder;

    private DigitalInput topLimitSwitch, bottomLimitSwitch;
    SparkFlexConfig configs = new SparkFlexConfig();

    private final TrapezoidProfile.Constraints elevatorConstraints = new TrapezoidProfile.Constraints(.3, .3); //TODO
    private ProfiledPIDController elevatorPid = new ProfiledPIDController(2.28, 0, 0, elevatorConstraints);

    private final TrapezoidProfile.Constraints arm1Constraints = new TrapezoidProfile.Constraints(.3, .3); //TODO
    private ProfiledPIDController shoulderPid = new ProfiledPIDController(5, 0, 1, arm1Constraints);

    private final TrapezoidProfile.Constraints wristConstraints = new TrapezoidProfile.Constraints(.3, .3); //TODO
    private ProfiledPIDController wristPid = new ProfiledPIDController(5, 0, 1, wristConstraints);

    //TODO
    private final ElevatorFeedforward elevatorFF = new ElevatorFeedforward(0, 2.28, 3.07, .41);
    private final ArmFeedforward shoulderFF = new ArmFeedforward(0,  1.75, 1.95); //ks is static friction, might not need it
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

    public ElevatorArmSubsystem(){
        elevatorMotorOne = new SparkFlex(RobotMap.MotorPorts.ELEVATOR_MOTOR_ONE, MotorType.kBrushless);
        elevatorMotorTwo = new SparkFlex(RobotMap.MotorPorts.ELEVATOR_MOTOR_TWO, MotorType.kBrushless);
        leftShoulderMotor = new SparkFlex(RobotMap.MotorPorts.LEFT_SHOULDER_MOTOR, MotorType.kBrushless);
        rightShoulderMotor = new SparkFlex(RobotMap.MotorPorts.RIGHT_SHOULDER_MOTOR, MotorType.kBrushless);
        wristMotor = new SparkFlex(RobotMap.MotorPorts.WRIST_MOTOR, MotorType.kBrushless);

        shoulderRelEncoder = leftShoulderMotor.getEncoder();
        elevatorEncoder = new DutyCycleEncoder(0, Math.PI, 0); // 0 to PI
        wristEncoder = new DutyCycleEncoder(1, Math.PI, 0);

        //limit switch, classic JJ
        topLimitSwitch = new DigitalInput(RobotMap.ArmConstants.TOP_LIMIT_SWITCH);
        bottomLimitSwitch = new DigitalInput(RobotMap.ArmConstants.BOTTOM_LIMIT_SWITCH);

        positionMap.put(ArmPosition.Starting, new double[] {0,0,0});
        positionMap.put(ArmPosition.L1, new double[] {0,0,0});
        positionMap.put(ArmPosition.L2, new double[] {0,0,0});
        positionMap.put(ArmPosition.L3, new double[] {0,0,0});
        positionMap.put(ArmPosition.L4, new double[] {0,0,0});
        positionMap.put(ArmPosition.Feeder, new double[] {0,0,0});
        positionMap.put(ArmPosition.Net, new double[] {0,0,0});

        setInverted(rightShoulderMotor);
        setInverted(elevatorMotorOne);
        // zeroShoulderRelEncoder();
        setElevatorArm(ArmPosition.Starting);
    }
    public void setInverted(SparkFlex motor){
        configs.inverted(true);
    }

    public void setManualElevator(double elevatorSpeed){
        elevatorMotorOne.set(elevatorSpeed);
        elevatorMotorTwo.set(elevatorSpeed);
    }
    public void setManualShoulder(double arm1Speed){
        leftShoulderMotor.set(arm1Speed);
        rightShoulderMotor.set(arm1Speed);
    }
    public void setManualWrist(double arm2Speed){wristMotor.set(arm2Speed);}


    // public void setSpeed(SparkFlex motor, double speed, double maxSpeed){
    //     double setSpeed = Math.max(-maxSpeed, Math.min(speed, maxSpeed));
    //     motor.set(setSpeed);
    // }

    public double getElevatorPos(){
        return elevatorEncoder.get();
    }
    // public double getShoulderPos(){
    //     return shoulderEncoder.get();
    // }
    public double getShoulderRelPos(){
        return shoulderRelEncoder.getPosition();
    }
    public double getWristPos(){
        return wristEncoder.get();
    }

    public void setElevatorArm(ArmPosition pos){
        double[] targets = positionMap.get(pos);
        elevatorTarget = targets[0];
        shoulderTarget = targets[1];
        wristTarget = targets[2];
    }

    // public void setElevator(double setpoint){
    //     double speed = elevatorPid.calculate(getElevatorPos(), setpoint);
    //     setSpeed(elevatorMotor, speed, RobotMap.ArmConstants.ELEVATOR_MAX_SPEED);
    // }


    public void stop(){
        elevatorMotorOne.set(0);
        elevatorMotorTwo.set(0);
        leftShoulderMotor.set(0);
        rightShoulderMotor.set(0);
        wristMotor.set(0);
    }

    public void zeroShoulderRelEncoder(){
        while(!topLimitSwitch.get()){
            leftShoulderMotor.set(.3);
            rightShoulderMotor.set(.3);
        }
        leftShoulderMotor.set(0);
        rightShoulderMotor.set(0);
        shoulderRelEncoder.setPosition(0);
    }

    public void moveElevator(double power){
        elevatorMotorOne.setVoltage(MathUtil.clamp(power, -12, 12));
        elevatorMotorTwo.setVoltage(MathUtil.clamp(power, -12, 12));
    }
    public void moveShoulder(double power){ 
        if((shoulderTarget > 90 || shoulderTarget < 0) && power > 0){ //TODO
            leftShoulderMotor.setVoltage(0);
            rightShoulderMotor.setVoltage(0);
            SmartDashboard.putBoolean("Shoulder: Too High or Too Low", true);
            return;
        }
        leftShoulderMotor.setVoltage(MathUtil.clamp(power, -12, 12));
        rightShoulderMotor.setVoltage(MathUtil.clamp(power, -12, 12));
        // SmartDashboard.putBoolean("Shoulder: Too High or Too Low", false);
    }
    public void moveWrist(double power){
        if((wristTarget > 90 || wristTarget < 0) && power > 0){ //TODO
            wristMotor.setVoltage(0);
            SmartDashboard.putBoolean("Wrist: Too High or Too Low", true);
            return;
        }
        wristMotor.setVoltage(MathUtil.clamp(power, -12, 12));
        // SmartDashboard.putBoolean("Shoulder: Too High or Too Low", false);
    }

    public boolean atTargetPosition(){
        boolean elevatorClose = Math.abs(getElevatorPos() - elevatorTarget) < 0.05;
    //    boolean shoulderClose = Math.abs(getShoulderPos() - shoulderTarget) < 0.05;
        boolean shoulderClose = Math.abs(getShoulderRelPos() - shoulderTarget) < 0.05;
        boolean wristClose = Math.abs(getWristPos() - wristTarget) < 0.05;
    
        SmartDashboard.putBoolean("Elevator At Target", elevatorClose);
        SmartDashboard.putBoolean("Shoulder At Target", shoulderClose);
        SmartDashboard.putBoolean("Wrist At Target", wristClose);
    
        return elevatorClose && shoulderClose && wristClose;
    }

    @Override
    public void periodic(){
         double elevatorPower = elevatorPid.calculate(getElevatorPos(), elevatorTarget);
        double shoulderPower = shoulderPid.calculate(getShoulderRelPos(), shoulderTarget) + shoulderFF.calculate(shoulderTarget, 0); // for limit switch
    //    double shoulderPower = shoulderPid.calculate(getShoulderPos(), shoulderTarget) + shoulderFF.calculate(shoulderTarget, 0); // add FF 
        double wristPower = wristPid.calculate(getWristPos(), wristTarget) + wristFF.calculate(wristTarget, 0); // add FF TODO

        moveElevator(elevatorPower);
        moveShoulder(shoulderPower);
        moveWrist(wristPower);

        if(topLimitSwitch.get()){
            shoulderRelEncoder.setPosition(0);
        }

        SmartDashboard.putNumber("Elevator Position", getElevatorPos());
    //    SmartDashboard.putNumber("Shoulder Position", getShoulderPos());
        SmartDashboard.putNumber("Shoulder Rel Pos", getShoulderRelPos());
        SmartDashboard.putNumber("Wrist Position", getWristPos());
     //   SmartDashboard.putNumber("Arm1 FF", shoulderFF.get());
    }
}
