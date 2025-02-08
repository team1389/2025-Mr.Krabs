// package frc.subsystems;

// import com.revrobotics.spark.SparkFlex;
// import com.revrobotics.spark.SparkLowLevel.MotorType;
// import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

// import edu.wpi.first.wpilibj2.command.SubsystemBase;
// import frc.robot.RobotMap;
// //landon motor systems are the best
// public class ClimberSubsystem extends SubsystemBase{
//     private SparkFlex spinnyMotor;

//     public ClimberSubsystem() {
//         spinnyMotor = new SparkFlex(RobotMap.MotorPorts.climberMotor, MotorType.kBrushless);
//     }

//     public void spinForwards(){
//         spinnyMotor.set(RobotMap.ClimberConstants.CLIMBER_SPEED);
//     }

//     public void spinBackwards(){
//         spinnyMotor.set(-RobotMap.ClimberConstants.CLIMBER_SPEED);
//     }

//     public void stop(){
//         spinnyMotor.set(0);
//     }
// }
