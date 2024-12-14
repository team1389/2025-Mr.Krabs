// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.RobotMap.OperatorConstants;
import frc.command.*;
import frc.subsystems.*;

import java.io.File;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class OI {
  //Controllers
  private final static CommandXboxController io_DriverController = new CommandXboxController(OperatorConstants.kDriverControllerPort);
  private final static CommandXboxController io_OperatorController = new CommandXboxController(OperatorConstants.kOperatorControllerPort);

  // Subsystems
  private final SwerveSubsystem  s_SwerveSubsystem = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(), "swerve"));

  //Commands


  //Auto
  private final SendableChooser<Command> autoChooser;
  
  /* The container for the robot. Contains subsystems, OI devices, and commands. */
  public OI() {
    registerCommands();
    autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Auto", autoChooser);
    
    setDefaultCommands();
    configureCamera();
    configureBindings();
  }

  private void registerCommands(){
    //put commands here to add to shuffleboard

  }

  private void setDefaultCommands(){
    //s_ArmSubsystem.setDefaultCommand(new ArmHoldAngle(s_ArmSubsystem, s_SwerveSubsystem, 17, 0.01)); TODO: Figure out if we want this
    //Run intake on operator's right stick

    Command driveFieldOrientedAnglularVelocity = s_SwerveSubsystem.driveCommand(
        () -> MathUtil.applyDeadband(-io_DriverController.getLeftY(), OperatorConstants.kDeadband),
        () -> MathUtil.applyDeadband(-io_DriverController.getLeftX(), OperatorConstants.kDeadband),
        () -> -MathUtil.applyDeadband(io_DriverController.getRightX(), OperatorConstants.kDeadband+0.05));
    s_SwerveSubsystem.setDefaultCommand(driveFieldOrientedAnglularVelocity);
  }

  private void configureCamera(){
    //Configure the USB camera here
    CameraServer.startAutomaticCapture();
    //UsbCamera intakeCam = CameraServer.startAutomaticCapture();
    //intakeCam.getActualDataRate(); <--Test this to see bandwidth usage
  }

  public static void setRightRumbleDriver(double rumble){
    io_DriverController.getHID().setRumble(RumbleType.kRightRumble, rumble);
  }
  public static void setRightRumbleOperator(double rumble){
    io_OperatorController.getHID().setRumble(RumbleType.kRightRumble, rumble);
  }

  private void configureBindings() {

    //Driver Controller
    io_DriverController.b().whileTrue( //Drive Slow button
    s_SwerveSubsystem.driveCommand(
        () -> MathUtil.applyDeadband(-io_DriverController.getLeftY()*.5, OperatorConstants.kDeadband),
        () -> MathUtil.applyDeadband(-io_DriverController.getLeftX()*.5, OperatorConstants.kDeadband),
        () -> -MathUtil.applyDeadband(io_DriverController.getRightX(), OperatorConstants.kDeadband+0.05))
    );
    io_DriverController.back().whileTrue(new InstantCommand(s_SwerveSubsystem::zeroGyro)); //Reset gyro
    

    ///Operator Controller
        //io_OperatorController.rightBumper().whileTrue(new ArmRotateDashboard(s_ArmSubsystem, 17, 0.01)); //for testing

    //io_OperatorController.start().whileTrue(z_ShootFullPower);
    //io_OperatorController.back().whileTrue(z_Shoot75Power);
    
  }
  private double getGreaterAxis(double axisOne, double axisTwo){
    if(Math.abs(axisOne) > Math.abs(axisTwo)){
      return axisOne;
    }else{
      return axisTwo;
    }
  }

  public Command getAutonomousCommand() {
    //return s_SwerveSubsystem.getAutonomousCommand("scoreClose3");
    return autoChooser.getSelected();
  }
}