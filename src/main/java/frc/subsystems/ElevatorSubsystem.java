// package frc.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;

public class ElevatorSubsystem extends SubsystemBase{
    private SparkFlex elevatorMotor;
    private RelativeEncoder elevatorRelativeEncoder;
    private SparkAbsoluteEncoder elevatorEncoder;
    private double elevatorSpeed = .3;
    // limit switch somewhere

    private SparkFlex arm1Motor;
    private double arm1Speed = .3;
    private SparkAbsoluteEncoder arm1Encoder;

    private SparkFlex arm2Motor;
    private double arm2Speed = .3;
    private SparkAbsoluteEncoder arm2Encoder;

//     private final TrapezoidProfile.Constraints elevatorConstraints = new TrapezoidProfile.Constraints(20, 10);

    public ElevatorSubsystem(){
        elevatorMotor = new SparkFlex(RobotMap.MotorPorts.ELEVATOR_MOTOR, MotorType.kBrushless);
        arm1Motor = new SparkFlex(RobotMap.MotorPorts.ARM_1_MOTOR, MotorType.kBrushless);
        arm2Motor = new SparkFlex(RobotMap.MotorPorts.ARM_2_MOTOR, MotorType.kBrushless);

        elevatorRelativeEncoder = elevatorMotor.getEncoder();
        elevatorEncoder = elevatorMotor.getAbsoluteEncoder();
        arm1Encoder = arm1Motor.getAbsoluteEncoder();
        arm2Encoder = arm2Motor.getAbsoluteEncoder();

        //limit switch on either side of arm 1 to zero, then relative encoder
        // classic JJ idea

        ElevatorFeedforward elevatorFF = new ElevatorFeedforward(0,0,0);
    }

    public void setManualElevator(double elevatorSpeed){elevatorMotor.set(elevatorSpeed);}
    public void setManualArm1(double arm1Speed){arm1Motor.set(arm1Speed);}
    public void setManualArm2(double arm2Speed){arm2Motor.set(arm2Speed);}

    public double getElevatorPos(){
        return elevatorEncoder.getPosition();
    }
    public double getArm1Pos(){
        return arm1Encoder.getPosition();
    }
    public double getArm2Pos(){
        return arm2Encoder.getPosition();
    }

    public void stop(){
        elevatorMotor.set(0);
        arm1Motor.set(0);
        arm2Motor.set(0);
    }

    @Override
    public void periodic(){
        SmartDashboard.putNumber("Elevator Position", getElevatorPos());
        SmartDashboard.putNumber("Arm1 Position", getArm1Pos());
        SmartDashboard.putNumber("Arm2 Position", getArm2Pos());
    }
}
