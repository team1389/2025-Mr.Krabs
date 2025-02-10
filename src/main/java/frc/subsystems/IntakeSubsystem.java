// package frc.subsystems;

// import com.revrobotics.spark.SparkFlex;
// import com.revrobotics.spark.SparkLowLevel.MotorType;

// import edu.wpi.first.wpilibj2.command.SubsystemBase;
// import frc.robot.RobotMap;

// public class IntakeSubsystem extends SubsystemBase {
//     private SparkFlex algaeIntake;
//     private SparkFlex coralIntake;
//     private final double speed = 0.5;

//     public IntakeSubsystem() {
//         algaeIntake = new SparkFlex(RobotMap.MotorPorts.intakeAlgaeMotor, MotorType.kBrushless);
//         coralIntake = new SparkFlex(RobotMap.MotorPorts.intakeCoralMotor, MotorType.kBrushless);
//     }
//     //Coral Intake
//     public void intakeCoral() {
//         coralIntake.set(speed);
//     }
//     //Coral Outtake
//     public void outtakeCoral() {
//         coralIntake.set(-speed);
//     }
//     //Algae Intake
//     public void intakeAlgae() {
//         algaeIntake.set(speed);
//     }
//     //Algae Outtake
//     public void outtakeAlgae() {
//         algaeIntake.set(-speed);
//     }
//     public void stopCoral() {
//         coralIntake.set(0);
//     }
//     public void stopAlgae() {
//         algaeIntake.set(0);
//     }
// }
