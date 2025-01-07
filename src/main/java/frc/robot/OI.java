// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.RobotMap.OperatorConstants;
import frc.subsystems.*;
import frc.command.*;

import java.io.File;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class OI {
  //Controllers
  private final static CommandXboxController driveController = new CommandXboxController(OperatorConstants.kDriverControllerPort);
  private final static CommandXboxController operatorController = new CommandXboxController(OperatorConstants.kOperatorControllerPort);

  // Subsystems
  private final SwerveSubsystem  swerveSubsystem = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(), "swerve"));

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
    //Run intake on operator's right stick

    Command driveFieldOrientedAnglularVelocity = swerveSubsystem.driveCommand(
        () -> MathUtil.applyDeadband(-driveController.getLeftY(), OperatorConstants.kDeadband),
        () -> MathUtil.applyDeadband(-driveController.getLeftX(), OperatorConstants.kDeadband),
        () -> -MathUtil.applyDeadband(driveController.getRightX(), OperatorConstants.kDeadband+0.05));
    swerveSubsystem.setDefaultCommand(driveFieldOrientedAnglularVelocity);
  }

  private void configureCamera(){
    //Configure the USB camera here
    CameraServer.startAutomaticCapture();
    //UsbCamera intakeCam = CameraServer.startAutomaticCapture();
    //intakeCam.getActualDataRate(); <--Test this to see bandwidth usage
  }

  public static void setRightRumbleDriver(double rumble){
    driveController.getHID().setRumble(RumbleType.kRightRumble, rumble);
  }
  public static void setRightRumbleOperator(double rumble){
    operatorController.getHID().setRumble(RumbleType.kRightRumble, rumble);
  }

  private void configureBindings() {

    //Driver Controller
    driveController.b().whileTrue( //Drive Slow button
    swerveSubsystem.driveCommand(
        () -> MathUtil.applyDeadband(-driveController.getLeftY()*.5, OperatorConstants.kDeadband),
        () -> MathUtil.applyDeadband(-driveController.getLeftX()*.5, OperatorConstants.kDeadband),
        () -> -MathUtil.applyDeadband(driveController.getRightX(), OperatorConstants.kDeadband+0.05))
    );
    driveController.back().whileTrue(new InstantCommand(swerveSubsystem::zeroGyro)); //Reset gyro
    

    ///Operator Controller
        //operatorController.rightBumper().whileTrue(new ArmRotateDashboard(s_ArmSubsystem, 17, 0.01)); //for testing

    //operatorController.start().whileTrue(z_ShootFullPower);
    //operatorController.back().whileTrue(z_Shoot75Power);
    
  }
  //I have no idea what this does -Zach
  private double getGreaterAxis(double axisOne, double axisTwo){
    if(Math.abs(axisOne) > Math.abs(axisTwo)){
      return axisOne;
    }else{
      return axisTwo;
    }
  }

  public Command getAutonomousCommand() {
    //return swerveSubsystem.getAutonomousCommand("scoreClose3");
    return autoChooser.getSelected();
  }
}