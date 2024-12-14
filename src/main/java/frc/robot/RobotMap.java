
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
  public static final double MAX_SPEED  = Units.feetToMeters(14.5);
  // Maximum speed of the robot in meters per second, used to limit acceleration.

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

  public static class OperatorConstants
  {

    // Joystick Deadband
    //tbd
    public static final double kDeadband = 0;

    public static final double LEFT_X_DEADBAND  = 0.1;
    public static final double LEFT_Y_DEADBAND  = 0.1;
    public static final double RIGHT_X_DEADBAND = 0.1;
    public static final double TURN_CONSTANT    = 6;
  }

  public static class SwerveConstants {
    //Speeds
    public static final double kMaxSpeed = Units.feetToMeters(17.2); // Maximum speed of the robot in meters per second, used to limit acceleration.
  }

  public static class AutoConstants {
    //PIDs
    public static final PIDConstants kTranslationPID = new PIDConstants(1.25, 0, 0); // was 0.7 before!
    public static final PIDConstants kAnglePID   = new PIDConstants(1, 0, 0.01); /// was 0.4 before!
  } 

  //Update the goal position for 2025
  public static class FieldConstants {
    public static final Translation2d kSpeakerPositionBLUE = new Translation2d(0.0, 5.547868); //TODO: Confirm these
    public static final Translation2d kSpeakerPositionRED = new Translation2d(16.5410642, 5.547868);
  }

}