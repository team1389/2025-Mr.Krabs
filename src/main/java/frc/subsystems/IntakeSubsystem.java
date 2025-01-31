package frc.subsystems;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeSubsystem extends SubsystemBase{
    private SparkFlex algaeIntake;
    private SparkFlex coralIntake;
    private final double spinnyspeed = 0.5;

    public IntakeSubsystem() {
        algaeIntake = new SparkFlex(1, MotorType.kBrushless);
        coralIntake = new SparkFlex(2, MotorType.kBrushless);
    }
    //Coral Intake
    public void suckIn() {
        coralIntake.set(spinnyspeed);
    }
    //Coral Outtake
    public void spitOut() {
        coralIntake.set(-spinnyspeed);
    }
    //Algae Intake
    public void injest() {
        algaeIntake.set(spinnyspeed);
    }
    //Algae Outtake
    public void regurgitate() {
        algaeIntake.set(-spinnyspeed);
    }
    public void stopCoral() {
        coralIntake.set(0);
    }
    public void stopAlgae() {
        algaeIntake.set(0);
    }
}
