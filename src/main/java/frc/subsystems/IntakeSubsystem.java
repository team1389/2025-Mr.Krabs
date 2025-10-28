package frc.subsystems;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLimitSwitch;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;

/**
 * The IntakeSubsystem class controls the intake mechanisms for both coral and algae.
 * It handles the motors and sensors used to detect and move game pieces.
 */
public class IntakeSubsystem extends SubsystemBase {

    // Motor that controls the algae intake (currently commented out)
    private SparkFlex algaeIntake;

    // Motor that controls the coral intake
    private SparkMax coralIntake;

    // Limit switch used to detect if a coral piece is present in the intake
    private SparkLimitSwitch coralLimitSwitch;

    // Default motor speed for intake/outtake operations
    double speed = 1;

    // Boolean flag to track whether coral is currently detected
    public boolean isCoralIn = false;

    // Unused digital inputs (for older hardware or future sensors)
    // private DigitalInput coralLS, algaeLS;

    /**
     * Constructor initializes the motors and sensors for the intake system.
     */
    public IntakeSubsystem() {
        // Initialize algae intake motor (currently disabled in code)
        // algaeIntake = new SparkFlex(RobotMap.MotorPorts.intakeAlgaeMotor, MotorType.kBrushless);

        // Initialize coral intake motor (brushless motor on defined port)
        coralIntake = new SparkMax(RobotMap.MotorPorts.intakeCoralMotor, MotorType.kBrushless);

        // Get the reverse limit switch attached to the coral intake motor
        coralLimitSwitch = coralIntake.getReverseLimitSwitch();
    }

    /**
     * Runs the coral intake motor forward to pull coral in.
     */
    public void intakeCoral() {
        coralIntake.set(speed);
    }

    /**
     * Runs the coral intake motor in reverse to push coral out.
     */
    public void outtakeCoral() {
        coralIntake.set(-speed);
    }

    /**
     * (Commented out) Runs the algae intake motor forward to pull algae in.
     */
    // public void intakeAlgae() {
    //     algaeIntake.set(speed);
    // }

    /**
     * (Commented out) Runs the algae intake motor in reverse to eject algae.
     */
    // public void outtakeAlgae() {
    //     algaeIntake.set(-speed);
    // }

    /**
     * Stops the coral intake motor.
     */
    public void stopCoral() {
        coralIntake.set(0);
    }

    /**
     * (Commented out) Stops the algae intake motor.
     */
    // public void stopAlgae() {
    //     algaeIntake.set(0);
    // }

    /**
     * Returns whether coral is currently detected in the intake.
     * 
     * @return true if coral is in, false otherwise
     */
    public boolean isCoralIn() {
        return isCoralIn;
    }

    /**
     * Periodic method runs every scheduler cycle (20ms by default).
     * Used here to continuously check the coral limit switch state
     * and update the isCoralIn variable.
     */
    @Override
    public void periodic() {
        // Update coral detection based on the limit switch
        isCoralIn = coralLimitSwitch.isPressed();

        // Optional: you could also send this data to the SmartDashboard for debugging
        // SmartDashboard.putBoolean("Coral Detected", isCoralIn);
    }
}
