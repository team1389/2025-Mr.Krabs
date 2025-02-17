package frc.subsystems;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLimitSwitch;
import com.revrobotics.spark.SparkMax;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;
//landon motor systems are the best
public class ArmTestSubsystem extends SubsystemBase{
    private SparkFlex leftArmMotor, rightArmMotor; //motor that moves intake
    private SparkFlex wristMotor; //motor that rotates the wrist
    private SparkMax algaeIntake; //motor that intakes algae
    private SparkMax coralIntake; //motor that intakes coral
    public double armMotorSpeed = 0.1;
    public double intakeCoralSpeed = 1;
    public double intakeAlgaeSpeed = 0.1;
    public double wristMotorSpeed = 0.1;
    private SparkLimitSwitch coralLimitSwitch;
    public boolean marikoSwitch;

    public ArmTestSubsystem() {
        leftArmMotor = new SparkFlex(RobotMap.MotorPorts.SHOULDER1_MOTOR, MotorType.kBrushless);
        rightArmMotor = new SparkFlex(RobotMap.MotorPorts.SHOULDER2_MOTOR, MotorType.kBrushless);
        wristMotor = new SparkFlex(RobotMap.MotorPorts.ELBOW_MOTOR, MotorType.kBrushless);
        algaeIntake = new SparkMax(RobotMap.MotorPorts.intakeAlgaeMotor, MotorType.kBrushless);
        coralIntake = new SparkMax(RobotMap.MotorPorts.intakeCoralMotor, MotorType.kBrushless);
        coralLimitSwitch = coralIntake.getReverseLimitSwitch();
        marikoSwitch = false;
        SmartDashboard.putBoolean("IsCoralIn:", marikoSwitch);
    }

    public void moveArmUp(){
        leftArmMotor.set(armMotorSpeed);
        rightArmMotor.set(-armMotorSpeed);
    }

    public void moveArmDown(){
        leftArmMotor.set(-armMotorSpeed);
        rightArmMotor.set(armMotorSpeed);
    }

    public void runAlgaeIntakeIn(){
        algaeIntake.set(intakeAlgaeSpeed);
    }

    public void runAlgaeIntakeOut(){
        algaeIntake.set(-intakeAlgaeSpeed);
    }

    public void runCoralIntakeIn(){
        marikoSwitch = !coralLimitSwitch.isPressed();
        SmartDashboard.putBoolean("IsCoralIn:", marikoSwitch);
        coralIntake.set(-intakeCoralSpeed);
    }

    public void runCoralIntakeOut(){
        marikoSwitch = !coralLimitSwitch.isPressed();
        SmartDashboard.putBoolean("IsCoralIn:", marikoSwitch);
        coralIntake.set(intakeCoralSpeed);
    }

    public void runWristForward(){
        wristMotor.set(wristMotorSpeed);
    }

    public void runWristBackwards(){
        wristMotor.set(-wristMotorSpeed);
    }

    public boolean getCoralInIntake(){
        return !coralLimitSwitch.isPressed();
    }

    public void stop(){
        leftArmMotor.set(0);
        rightArmMotor.set(0);
        wristMotor.set(0);
        algaeIntake.set(0);
        coralIntake.set(0);
    }
}