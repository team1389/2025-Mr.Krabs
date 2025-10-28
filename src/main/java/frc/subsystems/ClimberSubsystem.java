package frc.subsystems;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;

/**
 * The ClimberSubsystem controls the robot’s climbing mechanism.
 * It manages a single motor used to raise or lower the climber.
 */
public class ClimberSubsystem extends SubsystemBase {

    // Motor that powers the climbing system
    private SparkFlex climberMotor;

    /**
     * Constructor initializes the climber motor.
     */
    public ClimberSubsystem() {
        // Create a new SparkFlex motor on the specified port (brushless motor)
        climberMotor = new SparkFlex(RobotMap.MotorPorts.climberMotor, MotorType.kBrushless);
    }

    /**
     * Sets the speed of the climber motor.
     * @param climberSpeed Speed value between -1.0 (down) and 1.0 (up)
     */
    public void setSpeed(double climberSpeed) {
        climberMotor.set(climberSpeed);
    }

    /**
     * Stops the climber motor completely.
     */
    public void stop() {
        climberMotor.set(0);
    }
}
