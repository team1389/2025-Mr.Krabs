package frc.subsystems;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;

public class landonMotorSubsystem extends SubsystemBase{
    private SparkFlex spinnyMotor;

    public landonMotorSubsystem() {
        spinnyMotor = new SparkFlex(RobotMap.MotorPorts.landonMotor, MotorType.kBrushless);
    }

    public void spin(){
        spinnyMotor.set(1);
    }

    public void stop(){
        spinnyMotor.set(0);
    }

    // public int getEncoderValue(){
    //     return spinnyMotor.getEncoderPosition();
    // }
}
