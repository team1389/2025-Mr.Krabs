
package frc.robot;

import com.pathplanner.lib.config.PIDConstants;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import swervelib.math.Matter;

public final class RobotMap
{

  public static final double ROBOT_MASS = (148 - 20.3) * 0.453592; // 32lbs * kg per pound
  public static final Matter CHASSIS    = new Matter(new Translation3d(0, 0, Units.inchesToMeters(8)), ROBOT_MASS);
  public static final double LOOP_TIME  = 0.13; //s, 20ms + 110ms sprk max velocity lag
  public static final double MAX_SPEED  = Units.feetToMeters(14.5); //TODO: make it faster
  // Maximum speed of the robot in meters per second, used to limit acceleration.
public static final String INTAKE_MOTOR = null;

//  public static final class AutonConstants
//  {
//
//    public static final PIDConstants TRANSLATION_PID = new PIDConstants(0.7, 0, 0);
//    public static final PIDConstants ANGLE_PID       = new PIDConstants(0.4, 0, 0.01);
//  }

  public static final class DrivebaseConstants
  {

    // Hold time on motor brakes when disabled
    public static final double WHEEL_LOCK_TIME = 10; // seconds
  }

  public static final class ClimberConstants
  {
    public static final double CLIMBER_SPEED = 0.5;
  }

  public static final class ArmConstants
  {
    public static final double ELEVATOR_MAX_SPEED = 1;
    public static final double ARM1_MAX_SPEED = 1;
    public static final double ARM2_MAX_SPEED = 1;
  }

  public static class OperatorConstants
  {
    public static final int kDriverControllerPort = 0;
    public static final int kOperatorControllerPort = 1;
    // Joystick Deadband
    //tbd
    public static final double DEADBAND = 0.5;
    public static final double LEFT_X_DEADBAND  = 0.1;
    public static final double LEFT_Y_DEADBAND  = 0.1;
    public static final double RIGHT_X_DEADBAND = 0.1;
    public static final double TURN_CONSTANT    = 6;
  }
  public static class LimelightConstants {

    //TODO: All of these have to be determined later (some of these are useless tbh)
    public static final double AprilTagHeight = 0;
    public static final double LimelightHeight = 0;
    public static final double LimelightAngle = 0;
    public static final double SpeakerXDistfromCenter = 0;
    public static final double SpeakerYDistfromCenter = 0;
    public static final double XOffset = 0;
    public static final double YOffset = 0;
    public static final double dis_LL_to_bumpers = 0;

  }

  public static class MotorPorts{
    //TODO: add all motor ports
    public static final int climberMotor = 10;
    public static final int ELEVATOR_MOTOR = 11; 
    public static final int ARM_1_MOTOR = 12;
    public static final int ARM_2_MOTOR = 13;
    public static final int intakeAlgaeMotor = 14;
    public static final int intakeCoralMotor = 15;
  }
}