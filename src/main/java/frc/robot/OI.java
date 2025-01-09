// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.RobotMap.OperatorConstants;
import frc.subsystems.SwerveDrive.SwerveSubsystem;
import swervelib.SwerveInputStream;
import frc.command.*;
import frc.command.drivebase.AbsoluteDriveAdv;

import java.io.File;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class OI {
  //Controllers
  private final static CommandXboxController driveController = new CommandXboxController(OperatorConstants.kDriverControllerPort);
  private final static CommandXboxController operatorController = new CommandXboxController(OperatorConstants.kOperatorControllerPort);

  // Subsystems
  private final SwerveSubsystem  swerveSubsystem = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(), "swerve"));

  // Applies deadbands and inverts controls because joysticks
  // are back-right positive while robot
  // controls are front-left positive
  // left stick controls translation
  // right stick controls the rotational velocity 
  // buttons are quick rotation positions to different ways to face
  // WARNING: default buttons are on the same buttons as the ones defined in configureBindings
  AbsoluteDriveAdv closedAbsoluteDriveAdv = new AbsoluteDriveAdv(swerveSubsystem,
                                                                 () -> -MathUtil.applyDeadband(driveController.getLeftY(),
                                                                                               OperatorConstants.LEFT_Y_DEADBAND),
                                                                 () -> -MathUtil.applyDeadband(driveController.getLeftX(),
                                                                                               OperatorConstants.kDeadband),
                                                                 () -> -MathUtil.applyDeadband(driveController.getRightX(),
                                                                                               OperatorConstants.RIGHT_X_DEADBAND),
                                                                 driveController.getHID()::getYButtonPressed,
                                                                 driveController.getHID()::getAButtonPressed,
                                                                 driveController.getHID()::getXButtonPressed,
                                                                 driveController.getHID()::getBButtonPressed);


                                                                  /**
   * Converts driver input into a field-relative ChassisSpeeds that is controlled by angular velocity.
   */
  SwerveInputStream driveAngularVelocity = SwerveInputStream.of(swerveSubsystem.getSwerveDrive(),
                                                                () -> driveController.getLeftY() * -1,
                                                                () -> driveController.getLeftX() * -1)
                                                            .withControllerRotationAxis(driveController::getRightX)
                                                            .deadband(OperatorConstants.kDeadband)
                                                            .scaleTranslation(0.8)
                                                            .allianceRelativeControl(true);

  /**
   * Clone's the angular velocity input stream and converts it to a fieldRelative input stream.
   */
  SwerveInputStream driveDirectAngle = driveAngularVelocity.copy().withControllerHeadingAxis(driveController::getRightX,
                                                                                             driveController::getRightY)
                                                           .headingWhile(true);

                                                           
  // Applies deadbands and inverts controls because joysticks
  // are back-right positive while robot
  // controls are front-left positive
  // left stick controls translation
  // right stick controls the desired angle NOT angular rotation
  Command driveFieldOrientedDirectAngle = swerveSubsystem.driveFieldOriented(driveDirectAngle);

  // Applies deadbands and inverts controls because joysticks
  // are back-right positive while robot
  // controls are front-left positive
  // left stick controls translation
  // right stick controls the angular velocity of the robot
  Command driveFieldOrientedAnglularVelocity = swerveSubsystem.driveFieldOriented(driveAngularVelocity);

  Command driveSetpointGen = swerveSubsystem.driveWithSetpointGeneratorFieldRelative(driveDirectAngle);

  SwerveInputStream driveAngularVelocitySim = SwerveInputStream.of(swerveSubsystem.getSwerveDrive(),
                                                                   () -> -driveController.getLeftY(),
                                                                   () -> -driveController.getLeftX())
                                                               .withControllerRotationAxis(() -> driveController.getRawAxis(2))
                                                               .deadband(OperatorConstants.kDeadband)
                                                               .scaleTranslation(0.8)
                                                               .allianceRelativeControl(true);
  // Derive the heading axis with math! This is a simulation of a swerve drive with a gyro that can be controlled by the driver.
  SwerveInputStream driveDirectAngleSim     = driveAngularVelocitySim.copy()
                                                                     .withControllerHeadingAxis(() -> Math.sin(
                                                                                                    driveController.getRawAxis(
                                                                                                        2) * Math.PI) * (Math.PI * 2),
                                                                                                () -> Math.cos(
                                                                                                    driveController.getRawAxis(
                                                                                                        2) * Math.PI) *
                                                                                                      (Math.PI * 2))
                                                                     .headingWhile(true);

  Command driveFieldOrientedDirectAngleSim = swerveSubsystem.driveFieldOriented(driveDirectAngleSim);

  Command driveSetpointGenSim = swerveSubsystem.driveWithSetpointGeneratorFieldRelative(driveDirectAngleSim);
  //Commands


  //Auto
  private final SendableChooser<Command> autoChooser;
  
  /* The container for the robot. Contains subsystems, OI devices, and commands. */
  public OI() {
    registerCommands();
    autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Auto", autoChooser);
    
    // configureCamera();
    configureBindings();
    DriverStation.silenceJoystickConnectionWarning(true);
    NamedCommands.registerCommand("test", Commands.print("I EXIST"));
  }

  private void configureBindings()
  {
    // (Condition) ? Return-On-True : Return-on-False
    swerveSubsystem.setDefaultCommand(!RobotBase.isSimulation() ?
                                driveFieldOrientedDirectAngle :
                                driveFieldOrientedDirectAngleSim);

    if (Robot.isSimulation())
    {
      driveController.start().onTrue(Commands.runOnce(() -> swerveSubsystem.resetOdometry(new Pose2d(3, 3, new Rotation2d()))));
    }
    if (DriverStation.isTest())
    {
      swerveSubsystem.setDefaultCommand(driveFieldOrientedAnglularVelocity); // Overrides drive command above!

      driveController.b().whileTrue(swerveSubsystem.sysIdDriveMotorCommand());
      driveController.x().whileTrue(Commands.runOnce(swerveSubsystem::lock, swerveSubsystem).repeatedly());
      driveController.y().whileTrue(swerveSubsystem.driveToDistanceCommand(1.0, 0.2));
      driveController.start().onTrue((Commands.runOnce(swerveSubsystem::zeroGyro)));
      driveController.back().whileTrue(swerveSubsystem.centerModulesCommand());
      driveController.leftBumper().onTrue(Commands.none());
      driveController.rightBumper().onTrue(Commands.none());
    } else
    {
      driveController.a().onTrue((Commands.runOnce(swerveSubsystem::zeroGyro)));
      driveController.x().onTrue(Commands.none());
      driveController.b().whileTrue(
          swerveSubsystem.driveToPose(
              new Pose2d(new Translation2d(4, 4), Rotation2d.fromDegrees(0)))
                              );
      driveController.y().whileTrue(Commands.none());
      driveController.start().whileTrue(Commands.none());
      driveController.back().whileTrue(Commands.none());
      driveController.leftBumper().whileTrue(Commands.runOnce(swerveSubsystem::lock, swerveSubsystem).repeatedly());
      driveController.rightBumper().onTrue(Commands.none());
    }

  }

  public void setDriveMode()
  {
    configureBindings();
  }

  public void setMotorBrake(boolean brake)
  {
    swerveSubsystem.setMotorBrake(brake);
  }


  private void registerCommands(){
    //put commands here to add to shuffleboard

  }

  //I dont think we need this now
  //May have to uncomment later idk. (This is for running a camera)
  // private void configureCamera(){
  //   //Configure the USB camera here
  //   CameraServer.startAutomaticCapture();
  //   //UsbCamera intakeCam = CameraServer.startAutomaticCapture();
  //   //intakeCam.getActualDataRate(); <--Test this to see bandwidth usage
  // }

  public Command getAutonomousCommand() {
    //return swerveSubsystem.getAutonomousCommand("scoreClose3");
    return autoChooser.getSelected();
  }
}