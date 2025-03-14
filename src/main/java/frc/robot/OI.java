// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.File;
import java.util.function.BooleanSupplier;

import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.command.MoveClimber;
import frc.command.SetElevatorArm;
import frc.command.SetElevator;
import frc.command.SetShoulder;
import frc.command.SetWrist;
import frc.command.StartingPos;
import frc.command.TimedOuttakeCoral;
// import frc.command.AlignToReefTagRelative;
import frc.command.Feeder;
// import frc.command.IntakeAlgae;
import frc.command.IntakeCoral;
import frc.command.L2;
import frc.command.L3;
import frc.command.L4;
import frc.command.L4Action;
import frc.command.ManualElevatorArm;
// import frc.command.ManualElevatorArm;
// import frc.command.ManualElevatorArm;
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
 
  
  //Add auto options
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
                                                                () -> driveController.getLeftY() * -1,
                                                                () -> driveController.getLeftX() * -1) 
                                                                //Raw axis of rightTriggerAxis is 3 for some reason
                                                            .withControllerRotationAxis(() -> driveController.getRightTriggerAxis() * -1)
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

    //Auto Chooser
  SendableChooser<Command> m_chooser = new SendableChooser<>();
  //Auto
  private final Command m_simpleAuto = drivebase.getAutonomousCommand("Simple Single Piece Auto");
  private final Command m_simpleDualAuto = drivebase.getAutonomousCommand("Simple Dual Piece Auto");
  //set default option
  public OI()
  {
    // Configure the trigger bindings
    configureBindings();
    DriverStation.silenceJoystickConnectionWarning(true);
    NamedCommands.registerCommand("test", Commands.print("I EXIST"));
    NamedCommands.registerCommand("L4", new L4(elevatorArm));
    NamedCommands.registerCommand("WristL4", new SetWrist(elevatorArm, 265));
    NamedCommands.registerCommand("ShoulderL4", new SetShoulder(elevatorArm, -12.162));
    NamedCommands.registerCommand("ElevatorL4", new SetElevator(elevatorArm, 117.5555));

    NamedCommands.registerCommand("StartingPos", new StartingPos(elevatorArm));
    NamedCommands.registerCommand("Feeder", new Feeder(intake, elevatorArm));
    NamedCommands.registerCommand("Intake", new IntakeCoral(intake));
    NamedCommands.registerCommand("Outtake", new OuttakeCoral(intake));
    m_chooser.setDefaultOption("Simple Auto", m_simpleAuto);
    m_chooser.addOption("Simple Auto 2 Auto", m_simpleDualAuto);
  //post to smart dashboard
    SmartDashboard.putData(m_chooser);
  }


  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary predicate, or via the
   * named factories in {@link edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for
   * {@link CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller PS4}
   * controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight joysticks}.
   */

   Command zeroGyroCommand = Commands.runOnce(()-> drivebase.zeroGyro());
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
      driveController.a().onTrue((Commands.runOnce(drivebase::zeroGyro)));
      driveController.start().whileTrue(Commands.none());
      driveController.back().whileTrue(Commands.none());

      //should theoritically work. Untested with the modifications to how it is called.
      //UPADATE: IT DONT WORK
      // driveController.leftBumper().onTrue(drivebase.alignToReef(true));
      // driveController.rightBumper().onTrue(drivebase.alignToReef(false));


      // driveController.leftBumper().onTrue(new AlignToReefTagRelative(false, drivebase));
      // driveController.rightBumper().onTrue(new AlignToReefTagRelative(true, drivebase));
      // operatorController.pov(0).whileTrue(Commands.run(climber::spinForwards, climber));
      // operatorController.pov(180).whileTrue(Commands.run(climber::spinBackwards, climber));
      // operatorController.rightBumper().whileTrue(Commands.run(climber::spinForwards, climber));
      // operatorController.leftBumper().whileTrue(Commands.run(climber::spinBackwards, climber));
      // operatorController.x().whileTrue(Commands.run(climber::spinForwards, climber));
      // operatorController.rightBumper().whileTrue(new MoveClimber(climber, 1));
      // operatorController.leftBumper().whileTrue(new MoveClimber(climber, -1));

      // operatorController.x().whileTrue(new IntakeCoral(intake));
      // operatorController.b().whileTrue(new OuttakeCoral(intake));

      // operatorController.leftBumper().whileTrue(new RunManualShoulder(elevatorArm, 1));
      // operatorController.rightBumper().whileTrue(new RunManualShoulder(elevatorArm, -1));

      // operatorController.y().onTrue(new SetWrist(elevatorArm, 155.5));
      // operatorController.a().onTrue(new SetWrist(elevatorArm, 72.05));

      // operatorController.y().onTrue(new SetElevatorArm(elevatorArm, ArmPosition.Starting, 70, -10.01, 72.05));

      // operatorController.button(13).whileTrue(new MoveClimber(climber, 1)); //left trigger
      // operatorController.button(12).whileTrue(new MoveClimber(climber, -1)); //right trigger

      // operatorController.button(12).onTrue(new SetShoulder(elevatorArm, .2018)); //right trigger
      // operatorController.a().onTrue(new SetShoulder(elevatorArm, -12.162));

      // operatorController.y().onTrue(new SetWrist(elevatorArm, -20.25));
      // // operatorController.y().onTrue(new SetElevator(elevatorArm, 100));

      // operatorController.button(9).onTrue(new SetElevator(elevatorArm, 0.05)); //elippses
      // operatorController.button(10).onTrue(new SetElevator(elevatorArm, 70)); //menu

      elevatorArm.setDefaultCommand(new ManualElevatorArm(
        elevatorArm,
        () -> -getManipRightY(),
        () -> getManipLeftY()
      )
      );

      operatorController.x().onTrue(new SetElevator(elevatorArm, 117.5555));
      operatorController.x().onTrue(new SetWrist(elevatorArm, 265));
      operatorController.x().onTrue(new SetShoulder(elevatorArm, -12.162));

      operatorController.a().onTrue(new SetElevator(elevatorArm, .29577));
      operatorController.a().onTrue(new SetWrist(elevatorArm, 74.13));
      operatorController.a().onTrue(new SetShoulder(elevatorArm, .19503));

      //Micalea btn bindings
      // operatorController.x().onTrue(new L2(intake, elevatorArm));
      // operatorController.a().onTrue(new Feeder(intake, elevatorArm));
      operatorController.b().onTrue(new L4(elevatorArm));
      // operatorController.leftBumper().onTrue(new SetShoulder(elevatorArm, -10));
      // operatorController.rightBumper().onTrue(new SetElevator(elevatorArm, 70));

      operatorController.y().onTrue(new L3(intake, elevatorArm));
      // operatorController.button(9).onTrue(new L1(intake, elevatorArm)); // menu
      operatorController.button(14).onTrue(new StartingPos(elevatorArm));
      operatorController.button(12).onTrue(new MoveClimber(climber, 1)); //forward
      operatorController.button(13).onTrue(new MoveClimber(climber, -1));
      // operatorController.rightBumper().onTrue(new IntakeCoral(intake)); //right trigger
      // operatorController.leftBumper().onTrue(new OuttakeCoral(intake)); //left trigger
      
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

  public Command getZeroGyro(){
    return zeroGyroCommand;
  }
  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand()
  {
    // An example command will be run in autonomous
    return drivebase.getAutonomousCommand("Simple Single Piece Auto");
    // return m_chooser.getSelected();
  } 
  

  

  public void setMotorBrake(boolean brake)
  {
    drivebase.setMotorBrake(brake);
  }
}
