// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.File;
import java.util.function.BooleanSupplier;

import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.command.MoveClimber;
import frc.command.SetElevator;
import frc.command.SetShoulder;
import frc.command.SetWrist;
import frc.command.Starting;
import frc.command.Feeder;
import frc.command.IntakeCoral;
import frc.command.L2;
import frc.command.L3;
import frc.command.L4;
import frc.command.ManualElevatorArm;
import frc.robot.RobotMap.OperatorConstants;
import frc.subsystems.ClimberSubsystem;
import frc.subsystems.ElevatorArm;
import frc.subsystems.IntakeSubsystem;
import frc.subsystems.ElevatorArm.ArmPosition;
import frc.subsystems.SwerveSubsystem;
import swervelib.SwerveInputStream;
import frc.command.OuttakeCoral;
import frc.command.RunManualShoulder;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a "declarative" paradigm, very
 * little robot logic should actually be handled in the {@link Robot} periodic methods (other than the scheduler calls).
 * Instead, the structure of the robot (including subsystems, commands, and trigger mappings) should be declared here.
 */
public class OI
{

  // Replace with CommandPS4Controller or CommandJoystick if needed
  final         CommandXboxController driveController = new CommandXboxController(0);
  final        CommandXboxController operatorController = new CommandXboxController(1);
  // The robot's subsystems and commands are defined here...
  private final ElevatorArm elevatorArm = new ElevatorArm();
  private final ClimberSubsystem      climber    = new ClimberSubsystem();
  private final IntakeSubsystem intake = new IntakeSubsystem();
  private final SwerveSubsystem       drivebase  = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(),
                                                                                "swerve"));
                                                                                
                                                                              
  /**
   * Converts driver input into a field-relative ChassisSpeeds that is controlled by angular velocity.
   */
  SwerveInputStream driveAngularVelocity = SwerveInputStream.of(drivebase.getSwerveDrive(),
                                                                () -> driveController.getLeftY(),// * -1,
                                                                () -> driveController.getLeftX())// * -1) 
                                                                //possible change to getRightY if issue persists TODO: SEE IF IT WORKS with RightY
                                                                //Raw axis of rightTriggerAxis is 3 for some reason
                                                            .withControllerRotationAxis(() -> driveController.getRightTriggerAxis())
                                                            .deadband(OperatorConstants.DEADBAND)
                                                            .scaleTranslation(0.8)
                                                            .allianceRelativeControl(true);

  /**
   * Clone's the angular velocity input stream and converts it to a fieldRelative input stream.
   */
  SwerveInputStream driveDirectAngle = driveAngularVelocity.copy().withControllerHeadingAxis(driveController::getRightX,
                                                                                             driveController::getRightY)
                                                           .headingWhile(true);

  /**
   * Clone's the angular velocity input stream and converts it to a robotRelative input stream.
   */
  SwerveInputStream driveRobotOriented = driveAngularVelocity.copy().robotRelative(true)
                                                             .allianceRelativeControl(false);
  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public OI()
  {
    // Configure the trigger bindings
    configureBindings();
    DriverStation.silenceJoystickConnectionWarning(true);
    NamedCommands.registerCommand("test", Commands.print("I EXIST"));
  }


  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary predicate, or via the
   * named factories in {@link edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for
   * {@link CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller PS4}
   * controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight joysticks}.
   */
  private void configureBindings()
  {
    //RESERVE DRIVE B FOR AUTO ALIGN
    // Command driveFieldOrientedDirectAngle      = drivebase.driveFieldOriented(driveDirectAngle, () -> driveController.button(2).getAsBoolean());
    // Command driveFieldOrientedAnglularVelocity = drivebase.driveFieldOriented(driveAngularVelocity, () -> driveController.button(2).getAsBoolean());
    // Command driveRobotOrientedAngularVelocity  = drivebase.driveFieldOriented(driveRobotOriented, () -> driveController.button(2).getAsBoolean());
    Command driveFieldOrientedDirectAngle      = drivebase.driveFieldOriented(driveDirectAngle);
    Command driveFieldOrientedAnglularVelocity = drivebase.driveFieldOriented(driveAngularVelocity);
    Command driveRobotOrientedAngularVelocity  = drivebase.driveFieldOriented(driveRobotOriented);
    Command driveSetpointGen = drivebase.driveWithSetpointGeneratorFieldRelative(
        driveDirectAngle);

    drivebase.setDefaultCommand(driveFieldOrientedAnglularVelocity);
      //EDIT YOUR COMMANDS HERE_______________________________________________________________________________________________________________________________
      //dont use driver B for aything else, its already used for auto align
      driveController.a().onTrue((Commands.runOnce(drivebase::zeroGyro)));
      driveController.start().whileTrue(Commands.none());
      driveController.back().whileTrue(Commands.none());

      // driveController.x().onTrue(drivebase.driveToPose(new Pose2d(1, 1, new Rotation2d(0))));

      //Was creating a command of a command. Might work now. 
      driveController.leftBumper().onTrue(drivebase.alignToReef(true));
      driveController.rightBumper().onTrue(drivebase.alignToReef(false));

      operatorController.rightBumper().whileTrue(new IntakeCoral(intake));
      operatorController.leftBumper().whileTrue(new OuttakeCoral(intake));

      operatorController.button(13).whileTrue(new MoveClimber(climber, 1)); //left trigger
      operatorController.button(12).whileTrue(new MoveClimber(climber, -1)); //right trigger

      operatorController.b().onTrue(new L4(elevatorArm));

      // operatorController.b().onTrue(new SetShoulder(elevatorArm, -12.162)); //L4
      // operatorController.b().onTrue(new SetWrist(elevatorArm, 265));
      // operatorController.b().onTrue(new SetElevator(elevatorArm, 117.555));

      operatorController.y().onTrue(new L2(elevatorArm));

      // operatorController.y().onTrue(new SetShoulder(elevatorArm, .03976)); //L2
      // operatorController.y().onTrue(new SetWrist(elevatorArm, 79.09802));
      // operatorController.y().onTrue(new SetElevator(elevatorArm, 32.9788));

      operatorController.a().onTrue(new Feeder(elevatorArm));

      // operatorController.a().onTrue(new SetShoulder(elevatorArm, .19503)); //Feeder
      // operatorController.a().onTrue(new SetWrist(elevatorArm, 74.13));
      // operatorController.a().onTrue(new SetElevator(elevatorArm, 0.5177));

      operatorController.x().onTrue(new L3(elevatorArm));

      // operatorController.x().onTrue(new SetShoulder(elevatorArm, 0.0057)); //L3
      // operatorController.x().onTrue(new SetWrist(elevatorArm, 79.09802));
      // operatorController.x().onTrue(new SetElevator(elevatorArm, 71.4531));

      operatorController.button(9).onTrue(new Starting(elevatorArm));

      // operatorController.button(9).onTrue(new SetShoulder(elevatorArm, .208)); //starting
      // operatorController.button(9).onTrue(new SetWrist(elevatorArm, 62.95)); //ellipses
      // operatorController.button(9).onTrue(new SetElevator(elevatorArm, .5177));


 
      // elevatorArm.setDefaultCommand(new ManualElevatorArm(
      //   elevatorArm,
      //   () -> -getManipRightY(),
      //   () -> getManipLeftY(),
      //   () -> getManipGoogle(),
      //   () -> getManipFullscreen()
      // )
      // );


      elevatorArm.setDefaultCommand(new ManualElevatorArm(
        elevatorArm,
        () -> -getManipRightY(),
        () -> getManipLeftY()
      )
      );

      // operatorController.button(10).onTrue(new SetElevator(elevatorArm, 0)); //Menu
      // operatorController.button(10).onTrue(new SetElevator(elevatorArm, 70)); //menu
      // operatorController.button(14).onTrue(new SetElevatorArm(elevatorArm, ArmPosition.Starting)); //Google

  }

  public double getManipLeftY(){
    return operatorController.getRawAxis(1);
  }
  public double getManipRightY(){
    return operatorController.getRawAxis(4);
  }
  public double getManipRightTrigger(){
    return operatorController.getRightTriggerAxis();
  }
  public boolean getManipLeftTrigger(){
    return operatorController.leftTrigger().getAsBoolean();
  }
  public boolean getManipGoogle(){
    return operatorController.button(14).getAsBoolean();
  }
  public boolean getManipFullscreen(){
    return operatorController.button(15).getAsBoolean(); //TODO idk if it's 15
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand()
  {
    // An example command will be run in autonomous
    return drivebase.getAutonomousCommand("Tag7-18Left");
  }

  public void setMotorBrake(boolean brake)
  {
    drivebase.setMotorBrake(brake);
  }
}