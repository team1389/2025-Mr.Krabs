package frc.subsystems;
//lol
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;
//landon motor systems are the best
public class ClimberSubsystem extends SubsystemBase{
    private SparkFlex climberMotor;

    public ClimberSubsystem() {
        climberMotor = new SparkFlex(RobotMap.MotorPorts.climberMotor, MotorType.kBrushless);
    }

    public void setSpeed(double climberSpeed){
        climberMotor.set(climberSpeed);
    }
    public void stop(){
        climberMotor.set(0);
    }
}
