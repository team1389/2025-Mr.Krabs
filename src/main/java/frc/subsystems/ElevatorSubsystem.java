// package frc.subsystems;

// import com.revrobotics.RelativeEncoder;
// import com.revrobotics.spark.SparkFlex;
// import com.revrobotics.spark.SparkLowLevel.MotorType;

// import edu.wpi.first.math.trajectory.TrapezoidProfile;
// import edu.wpi.first.wpilibj.DutyCycleEncoder;
// import edu.wpi.first.wpilibj2.command.SubsystemBase;
// import frc.robot.RobotMap;

// public class ElevatorSubsystem extends SubsystemBase{
//     private SparkFlex elevatorMotor;
//     private RelativeEncoder elevatorRelativeEncoder;
//     private DutyCycleEncoder elevatorEncoder;
//     private double elevatorSpeed = .3;
//     // limit switch somewhere

//     private SparkFlex arm1Motor;
//     private double arm1Speed = .3;

//     private SparkFlex arm2Motor;
//     private double arm2Speed = .3;

//     private final TrapezoidProfile.Constraints elevatorConstraints = new TrapezoidProfile.Constraints(20, 10);

//     public ElevatorSubsystem(){
//         elevatorMotor = new SparkFlex(RobotMap.MotorPorts.ELEVATOR_MOTOR, MotorType.kBrushless);
//         arm1Motor = new SparkFlex(RobotMap.MotorPorts.ARM_1_MOTOR, MotorType.kBrushless);
//         arm2Motor = new SparkFlex(RobotMap.MotorPorts.ARM_2_MOTOR, MotorType.kBrushless);
//     }

//     public void setManualElevator(double elevatorSpeed){elevatorMotor.set(elevatorSpeed);}
//     public void setManualArm1(double arm1Speed){arm1Motor.set(arm1Speed);}
//     public void setManualArm2(double arm2Speed){arm2Motor.set(arm2Speed);}

//     public void stop(){
//         elevatorMotor.set(0);
//         arm1Motor.set(0);
//         arm2Motor.set(0);
//     }
// }
