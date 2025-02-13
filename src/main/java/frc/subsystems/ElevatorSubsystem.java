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

// public class ElevatorSubsystem extends SubsystemBase{
//     private SparkFlex elevatorMotor, arm1Motor, arm2Motor;

//     private RelativeEncoder elevatorRelativeEncoder;
//     private AbsoluteEncoder elevatorEncoder, arm1Encoder, arm2Encoder;

//     private DigitalInput topLimitSwitch, bottomLimitSwitch;

//     private final TrapezoidProfile.Constraints elevatorConstraints = new TrapezoidProfile.Constraints(20, 10); //TODO
//     private ProfiledPIDController elevatorPid = new ProfiledPIDController(0, 0, 0, elevatorConstraints);

//     private final TrapezoidProfile.Constraints arm1Constraints = new TrapezoidProfile.Constraints(20, 10); //TODO
//     private ProfiledPIDController arm1Pid = new ProfiledPIDController(0, 0, 0, arm1Constraints);

//     private final TrapezoidProfile.Constraints arm2Constraints = new TrapezoidProfile.Constraints(20, 10); //TODO
//     private ProfiledPIDController arm2Pid = new ProfiledPIDController(0, 0, 0, arm2Constraints);

//     //TODO
//     private final ElevatorFeedforward elevatorFF = new ElevatorFeedforward(0.1, 0.2, 0.1);
//     private final ArmFeedforward arm1FF = new ArmFeedforward(0.1, 0.2, 0.1);
//     private final ArmFeedforward arm2FF = new ArmFeedforward(0.1, 0.2, 0.1);

//     private double elevatorTarget, arm1Target, arm2Target;

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

//     public ElevatorSubsystem(){
//         elevatorMotor = new SparkFlex(RobotMap.MotorPorts.ELEVATOR_MOTOR, MotorType.kBrushless);
//         arm1Motor = new SparkFlex(RobotMap.MotorPorts.ARM_1_MOTOR, MotorType.kBrushless);
//         arm2Motor = new SparkFlex(RobotMap.MotorPorts.ARM_2_MOTOR, MotorType.kBrushless);

//         elevatorRelativeEncoder = elevatorMotor.getEncoder();
//         elevatorEncoder = elevatorMotor.getAbsoluteEncoder();
//         arm1Encoder = arm1Motor.getAbsoluteEncoder();
//         arm2Encoder = arm2Motor.getAbsoluteEncoder();

//         //limit switch, classic JJ idea
//         topLimitSwitch = new DigitalInput(1);
//         bottomLimitSwitch = new DigitalInput(0);

//         positionMap.put(ArmPosition.Starting, new double[] {0,0,0});
//         positionMap.put(ArmPosition.L1, new double[] {0,0,0});
//         positionMap.put(ArmPosition.L2, new double[] {0,0,0});
//         positionMap.put(ArmPosition.L3, new double[] {0,0,0});
//         positionMap.put(ArmPosition.L4, new double[] {0,0,0});
//         positionMap.put(ArmPosition.Feeder, new double[] {0,0,0});
//         positionMap.put(ArmPosition.Net, new double[] {0,0,0});

//         setElevatorArm(ArmPosition.Starting);

//     }

//     public void setManualElevator(double elevatorSpeed){elevatorMotor.set(elevatorSpeed);}
//     public void setManualArm1(double arm1Speed){arm1Motor.set(arm1Speed);}
//     public void setManualArm2(double arm2Speed){arm2Motor.set(arm2Speed);}


//     // public void setSpeed(SparkFlex motor, double speed, double maxSpeed){
//     //     double setSpeed = Math.max(-maxSpeed, Math.min(speed, maxSpeed));
//     //     motor.set(setSpeed);
//     // }

//     public double getElevatorPos(){
//         return elevatorEncoder.getPosition();
//     }
//     public double getArm1Pos(){
//         return arm1Encoder.getPosition();
//     }
//     public double getArm2Pos(){
//         return arm2Encoder.getPosition();
//     }

//     public void setElevatorArm(ArmPosition pos){
//         double[] targets = positionMap.get(pos);
//         elevatorTarget = targets[0];
//         arm1Target = targets[1];
//         arm2Target = targets[2];
//     }

//     // public void setElevator(double setpoint){
//     //     double speed = elevatorPid.calculate(getElevatorPos(), setpoint);
//     //     setSpeed(elevatorMotor, speed, RobotMap.ArmConstants.ELEVATOR_MAX_SPEED);
//     // }


//     public void stop(){
//         elevatorMotor.set(0);
//         arm1Motor.set(0);
//         arm2Motor.set(0);
//     }

//     public void moveElevator(double power){
//         elevatorMotor.setVoltage(MathUtil.clamp(power, -12, 12));
//     }
//     public void moveArm1(double power){
//         arm1Motor.setVoltage(MathUtil.clamp(power, -12, 12));
//     }
//     public void moveArm2(double power){
//         arm2Motor.setVoltage(MathUtil.clamp(power, -12, 12));
//     }

//     public boolean atTargetPosition(){
//         boolean elevatorClose = Math.abs(getElevatorPos() - elevatorTarget) < 0.05;
//         boolean arm1Close = Math.abs(getArm1Pos() - arm1Target) < 0.05;
//         boolean arm2Close = Math.abs(getArm2Pos() - arm2Target) < 0.05;
    
//         SmartDashboard.putBoolean("Elevator At Target", elevatorClose);
//         SmartDashboard.putBoolean("Arm1 At Target", arm1Close);
//         SmartDashboard.putBoolean("Arm2 At Target", arm2Close);
    
//         return elevatorClose && arm1Close && arm2Close;
//     }

//     @Override
//     public void periodic(){
//         double elevatorPower = elevatorPid.calculate(getElevatorPos(), elevatorTarget);
//         double arm1Power = arm1Pid.calculate(getArm1Pos(), arm1Target) + arm1FF.calculate(arm1Target, 0); // add FF TODO
//         double arm2Power = arm2Pid.calculate(getArm2Pos(), arm2Target) + arm2FF.calculate(arm2Target, 0); // add FF TODO

//         moveElevator(elevatorPower);
//         moveArm1(arm1Power);
//         moveArm2(arm2Power);

//         SmartDashboard.putNumber("Elevator Position", getElevatorPos());
//         SmartDashboard.putNumber("Arm1 Position", getArm1Pos());
//         SmartDashboard.putNumber("Arm2 Position", getArm2Pos());
//      //   SmartDashboard.putNumber("Arm1 FF", arm1FF.get());
//     }
// }
