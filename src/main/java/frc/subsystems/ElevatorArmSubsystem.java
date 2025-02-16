// package frc.subsystems;

// import java.util.HashMap;
// import java.util.Map;

// import com.revrobotics.AbsoluteEncoder;
// import com.revrobotics.RelativeEncoder;
// import com.revrobotics.spark.SparkAbsoluteEncoder;
// import com.revrobotics.spark.SparkFlex;
// import com.revrobotics.spark.SparkLowLevel.MotorType;
// import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

// import edu.wpi.first.math.MathUtil;
// import edu.wpi.first.math.controller.ArmFeedforward;
// import edu.wpi.first.math.controller.ElevatorFeedforward;
// import edu.wpi.first.math.controller.PIDController;
// import edu.wpi.first.math.controller.ProfiledPIDController;
// import edu.wpi.first.math.trajectory.TrapezoidProfile;
// import edu.wpi.first.wpilibj.DigitalInput;
// import edu.wpi.first.wpilibj.DutyCycleEncoder;
// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// import edu.wpi.first.wpilibj2.command.SubsystemBase;
// import frc.robot.RobotMap;

// public class ElevatorArmSubsystem extends SubsystemBase{
//     private SparkFlex elevatorMotor, shoulder1Motor, shoulder2Motor, elbowMotor;

//     private RelativeEncoder shoulderRelEncoder;
//     private AbsoluteEncoder elevatorEncoder, shoulderEncoder, elbowEncoder;

//     private DigitalInput topLimitSwitch, bottomLimitSwitch;

//     private final TrapezoidProfile.Constraints elevatorConstraints = new TrapezoidProfile.Constraints(20, 10); //TODO
//     private ProfiledPIDController elevatorPid = new ProfiledPIDController(0, 0, 0, elevatorConstraints);

//     private final TrapezoidProfile.Constraints arm1Constraints = new TrapezoidProfile.Constraints(20, 10); //TODO
//     private ProfiledPIDController shoulderPid = new ProfiledPIDController(0, 0, 0, arm1Constraints);

//     private final TrapezoidProfile.Constraints elbowConstraints = new TrapezoidProfile.Constraints(20, 10); //TODO
//     private ProfiledPIDController elbowPid = new ProfiledPIDController(0, 0, 0, elbowConstraints);

//     //TODO
//     private final ElevatorFeedforward elevatorFF = new ElevatorFeedforward(0.1, 0.2, 0.1);
//     private final ArmFeedforward shoulderFF = new ArmFeedforward(0.1, 0.2, 0.1);
//     private final ArmFeedforward elbowFF = new ArmFeedforward(0.1, 0.2, 0.1); 

//     private double elevatorTarget, shoulderTarget, elbowTarget;

//     public enum ArmPosition {
//         Starting,
//         L1,
//         L2,
//         L3,
//         L4,
//         Feeder,
//         Net
//     }

//     private final Map<ArmPosition, double[]> positionMap = new HashMap<>();

//     public ElevatorArmSubsystem(){
//         elevatorMotor = new SparkFlex(RobotMap.MotorPorts.ELEVATOR_MOTOR, MotorType.kBrushless);
//         shoulder1Motor = new SparkFlex(RobotMap.MotorPorts.SHOULDER1_MOTOR, MotorType.kBrushless);
//         shoulder2Motor = new SparkFlex(RobotMap.MotorPorts.SHOULDER2_MOTOR, MotorType.kBrushless);
//         elbowMotor = new SparkFlex(RobotMap.MotorPorts.ELBOW_MOTOR, MotorType.kBrushless);

//         shoulderRelEncoder = shoulder1Motor.getEncoder();
//         elevatorEncoder = elevatorMotor.getAbsoluteEncoder();
//         shoulderEncoder = shoulder1Motor.getAbsoluteEncoder();
//         elbowEncoder = elbowMotor.getAbsoluteEncoder();

//         //limit switch, classic JJ
//         topLimitSwitch = new DigitalInput(RobotMap.ArmConstants.TOP_LIMIT_SWITCH);
//         bottomLimitSwitch = new DigitalInput(0);

//         positionMap.put(ArmPosition.Starting, new double[] {0,0,0});
//         positionMap.put(ArmPosition.L1, new double[] {0,0,0});
//         positionMap.put(ArmPosition.L2, new double[] {0,0,0});
//         positionMap.put(ArmPosition.L3, new double[] {0,0,0});
//         positionMap.put(ArmPosition.L4, new double[] {0,0,0});
//         positionMap.put(ArmPosition.Feeder, new double[] {0,0,0});
//         positionMap.put(ArmPosition.Net, new double[] {0,0,0});

//         // zeroShoulderRelEncoder();
//         setElevatorArm(ArmPosition.Starting);
//     }

//     public void setManualElevator(double elevatorSpeed){elevatorMotor.set(elevatorSpeed);}
//     public void setManualShoulder(double arm1Speed){
//         shoulder1Motor.set(arm1Speed);
//         shoulder2Motor.set(arm1Speed);
//     }
//     public void setManualElbow(double arm2Speed){elbowMotor.set(arm2Speed);}


//     // public void setSpeed(SparkFlex motor, double speed, double maxSpeed){
//     //     double setSpeed = Math.max(-maxSpeed, Math.min(speed, maxSpeed));
//     //     motor.set(setSpeed);
//     // }

//     public double getElevatorPos(){
//         return elevatorEncoder.getPosition();
//     }
//     public double getShoulderPos(){
//         return shoulderEncoder.getPosition();
//     }
//     public double getShoulderRelPos(){
//         return shoulderRelEncoder.getPosition();
//     }
//     public double getElbowPos(){
//         return elbowEncoder.getPosition();
//     }

//     public void setElevatorArm(ArmPosition pos){
//         double[] targets = positionMap.get(pos);
//         elevatorTarget = targets[0];
//         shoulderTarget = targets[1];
//         elbowTarget = targets[2];
//     }

//     // public void setElevator(double setpoint){
//     //     double speed = elevatorPid.calculate(getElevatorPos(), setpoint);
//     //     setSpeed(elevatorMotor, speed, RobotMap.ArmConstants.ELEVATOR_MAX_SPEED);
//     // }


//     public void stop(){
//         elevatorMotor.set(0);
//         shoulder1Motor.set(0);
//         shoulder2Motor.set(0);
//         elbowMotor.set(0);
//     }

//     public void zeroShoulderRelEncoder(){
//         while(!topLimitSwitch.get()){
//             shoulder1Motor.set(.3);
//             shoulder2Motor.set(.3);
//         }
//         shoulder1Motor.set(0);
//         shoulder2Motor.set(0);
//         shoulderRelEncoder.setPosition(0);
//     }

//     public void moveElevator(double power){
//         elevatorMotor.setVoltage(MathUtil.clamp(power, -12, 12));
//     }
//     public void moveShoulder(double power){
//         shoulder1Motor.setVoltage(MathUtil.clamp(power, -12, 12));
//         shoulder2Motor.setVoltage(MathUtil.clamp(power, -12, 12));
//     }
//     public void moveElbow(double power){
//         elbowMotor.setVoltage(MathUtil.clamp(power, -12, 12));
//     }

//     public boolean atTargetPosition(){
//         boolean elevatorClose = Math.abs(getElevatorPos() - elevatorTarget) < 0.05;
//     //    boolean shoulderClose = Math.abs(getShoulderPos() - shoulderTarget) < 0.05;
//         boolean shoulderClose = Math.abs(getShoulderRelPos() - shoulderTarget) < 0.05;
//         boolean elbowClose = Math.abs(getElbowPos() - elbowTarget) < 0.05;
    
//         SmartDashboard.putBoolean("Elevator At Target", elevatorClose);
//         SmartDashboard.putBoolean("Shoulder At Target", shoulderClose);
//         SmartDashboard.putBoolean("Elbow At Target", elbowClose);
    
//         return elevatorClose && shoulderClose && elbowClose;
//     }

//     @Override
//     public void periodic(){
//         // double elevatorPower = elevatorPid.calculate(getElevatorPos(), elevatorTarget);
//         double shoulderPower = shoulderPid.calculate(getShoulderRelPos(), shoulderTarget) + shoulderFF.calculate(shoulderTarget, 0); // for limit switch
//     //    double shoulderPower = shoulderPid.calculate(getShoulderPos(), shoulderTarget) + shoulderFF.calculate(shoulderTarget, 0); // add FF TODO shoulderTarget needs to be in radians
//         double elbowPower = elbowPid.calculate(getElbowPos(), elbowTarget) + elbowFF.calculate(elbowTarget, 0); // add FF TODO

//       //  moveElevator(elevatorPower);
//         moveShoulder(shoulderPower);
//         moveElbow(elbowPower);

//         SmartDashboard.putNumber("Elevator Position", getElevatorPos());
//         SmartDashboard.putNumber("Shoulder Position", getShoulderPos());
//         SmartDashboard.putNumber("Shoulder Rel Pos", getShoulderRelPos());
//         SmartDashboard.putNumber("Elbow Position", getElbowPos());
//      //   SmartDashboard.putNumber("Arm1 FF", shoulderFF.get());
//     }
// }
