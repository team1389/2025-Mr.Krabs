package frc.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLimitSwitch;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkRelativeEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;
//landon motor systems are the best
public class ArmTestSubsystem extends SubsystemBase{
    private SparkFlex leftShoulderMotor, rightShoulderMotor; //motor that moves intake
    private SparkFlex wristMotor; //motor that rotates the wrist
    private SparkMax algaeIntake; //motor that intakes algae
    private SparkMax coralIntake; //motor that intakes coral
    public double armMotorSpeed = 0.1;
    public double intakeCoralSpeed = 1;
    public double intakeAlgaeSpeed = 0.1;
    public double wristMotorSpeed = 0.1;
    private SparkLimitSwitch coralLimitSwitch;
    private DutyCycleEncoder wristEncoder;
    private DigitalInput leftLimitSwitch, rightLimitSwitch;
    //marikoSwitch is a boolean that is true when the coral is in the intake
    public boolean marikoSwitch;
    private RelativeEncoder leftShoulderEncoder, rightShoulderEncoder;
    //TODO: FIND ACTUAL VALUES FOR wristMax and wristMin
    public final double wristMax = 0.7;
    public final double wristMin = 0.3;
    public boolean isLeftLimitSwitch, isRightLimitSwitch;

    public ArmTestSubsystem() {
        leftShoulderMotor = new SparkFlex(RobotMap.MotorPorts.SHOULDER1_MOTOR, MotorType.kBrushless);
        rightShoulderMotor = new SparkFlex(RobotMap.MotorPorts.SHOULDER2_MOTOR, MotorType.kBrushless);
        wristMotor = new SparkFlex(RobotMap.MotorPorts.ELBOW_MOTOR, MotorType.kBrushless);
        algaeIntake = new SparkMax(RobotMap.MotorPorts.intakeAlgaeMotor, MotorType.kBrushless);
        coralIntake = new SparkMax(RobotMap.MotorPorts.intakeCoralMotor, MotorType.kBrushless);
        coralLimitSwitch = coralIntake.getReverseLimitSwitch();
        leftShoulderEncoder = leftShoulderMotor.getEncoder();
        rightShoulderEncoder = rightShoulderMotor.getEncoder();
        marikoSwitch = false;
        SmartDashboard.putBoolean("IsCoralIn:", marikoSwitch);
        //TODO:get actual port from landon
        wristEncoder = new DutyCycleEncoder(9);
        leftLimitSwitch = new DigitalInput(8);
        rightLimitSwitch = new DigitalInput(7);
        isLeftLimitSwitch = leftLimitSwitch.get();
        isRightLimitSwitch = rightLimitSwitch.get();
        SmartDashboard.putNumber("WristAngle:", wristEncoder.get());
        SmartDashboard.putBoolean("isLeftLimitSwitch:", isLeftLimitSwitch);
        SmartDashboard.putBoolean("isRightLimitSwitch:", isRightLimitSwitch);
    }

    public void moveArmUp(){
        isLeftLimitSwitch = leftLimitSwitch.get();
        isRightLimitSwitch = rightLimitSwitch.get();
        SmartDashboard.putBoolean("isLeftLimitSwitch:", isLeftLimitSwitch);
        SmartDashboard.putBoolean("isRightLimitSwitch:", isRightLimitSwitch);
        leftShoulderMotor.set(armMotorSpeed);
        rightShoulderMotor.set(-armMotorSpeed);
    }

    public void moveArmDown(){
        isLeftLimitSwitch = leftLimitSwitch.get();
        isRightLimitSwitch = rightLimitSwitch.get();
        SmartDashboard.putBoolean("isLeftLimitSwitch:", isLeftLimitSwitch);
        SmartDashboard.putBoolean("isRightLimitSwitch:", isRightLimitSwitch);
        leftShoulderMotor.set(-armMotorSpeed);
        rightShoulderMotor.set(armMotorSpeed);
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

    public boolean getIsCoralInIntake(){
        return marikoSwitch;
    }

    public void runWristForward(){
        if (wristMax>wristEncoder.get()){
            wristMotor.set(wristMotorSpeed);
        }
        else{
            wristMotor.set(0);
        }
    }

    public void runWristBackwards(){
        if (wristMin<wristEncoder.get()){
            wristMotor.set(-wristMotorSpeed);
        }
        else{
            wristMotor.set(0);
        }
    }

    

    public void stop(){
        leftShoulderMotor.set(0);
        rightShoulderMotor.set(0);
        wristMotor.set(0);
        algaeIntake.set(0);
        coralIntake.set(0);
    }
}