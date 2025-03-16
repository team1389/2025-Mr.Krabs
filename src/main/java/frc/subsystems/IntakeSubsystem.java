package frc.subsystems;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLimitSwitch;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;

public class IntakeSubsystem extends SubsystemBase {
    private SparkFlex algaeIntake;
    private SparkMax coralIntake;
    private SparkLimitSwitch coralLimitSwitch;
    double speed = 1;
    public boolean isCoralIn = false;
    // private DigitalInput coralLS, algaeLS;


    public IntakeSubsystem() {
        // algaeIntake = new SparkFlex(RobotMap.MotorPorts.intakeAlgaeMotor, MotorType.kBrushless);
        coralIntake = new SparkMax(RobotMap.MotorPorts.intakeCoralMotor, MotorType.kBrushless);
        coralLimitSwitch = coralIntake.getReverseLimitSwitch();

    }
    //Coral Intake
    public void intakeCoral() {
        coralIntake.set(speed);
    }
    //Coral Outtake
    public void outtakeCoral() {
        coralIntake.set(-speed);
    }
    //Algae Intake
    // public void intakeAlgae() {
    //     algaeIntake.set(speed);
    // }
    // //Algae Outtake
    // public void outtakeAlgae() {
    //     algaeIntake.set(-speed);
    // }
    public void stopCoral() {
        coralIntake.set(0);
    }
    // public void stopAlgae() {
    //     algaeIntake.set(0);
    // }

    public boolean isCoralIn() {
        return isCoralIn;
    }

    @Override
    public void periodic(){
        isCoralIn = coralLimitSwitch.isPressed();
    }
}
