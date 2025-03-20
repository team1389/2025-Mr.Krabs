
package frc.robot;

import com.pathplanner.lib.config.PIDConstants;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import swervelib.math.Matter;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;

public final class RobotMap
{

  public static final double ROBOT_MASS = 61.235;
  public static final Matter CHASSIS    = new Matter(new Translation3d(0, 0, Units.inchesToMeters(8)), ROBOT_MASS);
  public static final double LOOP_TIME  = 0.13; //s, 20ms + 110ms sprk max velocity lag
  public static final double MAX_SPEED  = 5; //IN METERS TODO: FIND MAX ACTUAL SPEED!
  public static final double fieldLength   = Units.inchesToMeters(690.876);
  public static final double fieldWidth    = Units.inchesToMeters(317);
  public static final double startingLineX =
      Units.inchesToMeters(299.438); // Measured from the inside of starting line
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

  public static class Left
      {

        public static final Transform2d offset = new Transform2d(Inches.of(24).in(Meters),
                                                                 Inches.of(0).in(Meters),
                                                                 Rotation2d.fromDegrees(0));
      }

  public static final class ClimberConstants
  {
    public static final double CLIMBER_SPEED = .5;
  }
  public static final class IntakeConstants
  {
    public static final int ALGAE_LIMIT_SWITCH = 0;
    public static final int CORAL_LIMIT_SWITCH = 1;
  }

  public static final class ArmConstants
  {
    public static final double ELEVATOR_MAX_SPEED = 1;
    public static final double ARM1_MAX_SPEED = 1;
    public static final double ARM2_MAX_SPEED = 1;
    public static final int TOP_LIMIT_SWITCH = 0;
    public static final int BOTTOM_LIMIT_SWITCH = 1;

    public static final double ElevatorP = 3;
    public static final double ElevatorI = 0; //1
    public static final double ElevatorD = .05; //.05
    public static final double ElevatorMaxVelocity = 40000;//Meters.of(.42).per(Second).in(MetersPerSecond);// prev 40000
    public static final double ElevatorMaxAccerlation = 62000;//Meters.of(10.34).per(Second).per(Second).in(MetersPerSecondPerSecond);// prev 62000

    public static final double ElevatorS = .02; //.02
    public static final double ElevatorG = .05;          //prev .9
    public static final double ElevatorV = 21.28; //3.8   //prev 3.8
    public static final double ElevatorA = .01; //.17   // prev .17
    public static final double ElevatorRampRate = .1;

    public static final double ElevatorGearing = 21.3;
    public static final double ElevatorCarriageMass = 6.8;
    public static final double ElevatorDrumRadius = Units.inchesToMeters(1);
    public static final double ElevatorMinHeight = Units.inchesToMeters(54); //meters
    public static final double ElevatorMaxHeight = Units.inchesToMeters(80);

    public static final double WristGearRatio = 48;

  }

  public static class OperatorConstants
  {
    public static final int kDriverControllerPort = 0;
    public static final int kOperatorControllerPort = 1;
    // Joystick Deadband
    //tbd
    public static final double DEADBAND = 0.25;
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
    public static final int ELEVATOR_MOTOR_ONE = 11; 
    public static final int ELEVATOR_MOTOR_TWO = 17; //right
    public static final int LEFT_SHOULDER_MOTOR = 12;
    public static final int RIGHT_SHOULDER_MOTOR = 13;
    public static final int WRIST_MOTOR = 14;
    // public static final int intakeAlgaeMotor = 15;
    public static final int intakeCoralMotor = 16;
  }

  public static class CoralStation {
    public static final double fieldLength   = Units.inchesToMeters(690.876);
    public static final double fieldWidth    = Units.inchesToMeters(317);
    public static final double stationLength = Units.inchesToMeters(79.750);
    public static final Pose2d rightCenterFace =
        new Pose2d(
            Units.inchesToMeters(33.526),
            Units.inchesToMeters(25.824),
            Rotation2d.fromDegrees(144.011 - 90));
    public static final Pose2d leftCenterFace =
        new Pose2d(
            rightCenterFace.getX(),
            fieldWidth - rightCenterFace.getY(),
            Rotation2d.fromRadians(-rightCenterFace.getRotation().getRadians()));
  }
}