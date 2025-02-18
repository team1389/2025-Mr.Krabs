// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.File;

import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.command.runClimberDown;
import frc.command.runClimberUp;
// import frc.command.SetElevatorArm;
// import frc.command.ManualElevator;
import frc.robot.RobotMap.OperatorConstants;
import frc.subsystems.ClimberSubsystem;
// import frc.subsystems.ElevatorArmSubsystem;
// import frc.subsystems.ElevatorArmSubsystem.ArmPosition;
import frc.subsystems.SwerveSubsystem;
import swervelib.SwerveInputStream;
import frc.subsystems.ArmTestSubsystem;

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
  private final ClimberSubsystem      climber    = new ClimberSubsystem();
  // private final ElevatorArmSubsystem elevator = new ElevatorArmSubsystem();
  // private final IntakeSubsystem intake = new IntakeSubsystem();
  private final SwerveSubsystem       drivebase  = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(),
                                                                                "swerve"));
  private final ArmTestSubsystem armTest = new ArmTestSubsystem();
                                                                                
             //name is makala                                                                 
  /**
   * Converts driver input into a field-relative ChassisSpeeds that is controlled by angular velocity.
   */
  SwerveInputStream driveAngularVelocity = SwerveInputStream.of(drivebase.getSwerveDrive(),
                                                                () -> driveController.getLeftY() * -1,
                                                                () -> driveController.getLeftX() * -1)
                                                                //possible change to getRightY if issue persists TODO: SEE IF IT WORKS with RightY
                                                                //Raw axis of rightBumperAxis is 3 for some reason
                                                            .withControllerRotationAxis(driveController::getRightTriggerAxis)
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
    // //RESERVE DRIVE B FOR AUTO ALIGN
    Command driveFieldOrientedDirectAngle      = drivebase.driveFieldOriented(driveDirectAngle, () -> driveController.b().getAsBoolean());
    Command driveFieldOrientedAnglularVelocity = drivebase.driveFieldOriented(driveAngularVelocity, () -> driveController.b().getAsBoolean());
    Command driveRobotOrientedAngularVelocity  = drivebase.driveFieldOriented(driveRobotOriented, () -> driveController.b().getAsBoolean());
    Command driveSetpointGen = drivebase.driveWithSetpointGeneratorFieldRelative(
        driveDirectAngle);

      //EDIT YOUR COMMANDS HERE_______________________________________________________________________________________________________________________________
      //dont use driver B for aything else, its already used for auto align

      //DRIVER CODE
      drivebase.setDefaultCommand(driveFieldOrientedAnglularVelocity);
      driveController.a().onTrue((Commands.runOnce(drivebase::zeroGyro)));
      driveController.start().whileTrue(Commands.none());
      driveController.back().whileTrue(Commands.none());
      driveController.leftBumper().whileTrue(Commands.runOnce(drivebase::lock, drivebase).repeatedly());
      driveController.rightBumper().onTrue(Commands.none());
      




      //MANIP CODE
      // operatorController.a().whileTrue(new runClimberDown(climber));
      // operatorController.y().whileTrue(new runClimberUp(climber));

      // operatorController.x().whileTrue(new IntakeAlgae(intake));
      // operatorController.y().whileTrue(new IntakeCoral(intake));
      // operatorController.pov(0).whileTrue(Commands.run(climber::spinForwards, climber));
      // operatorController.pov(180).whileTrue(Commands.run(climber::spinBackwards, climber));
      // operatorController.a().whileTrue(Commands.run(climber::spinForwards, climber));
      // operatorController.y().whileTrue(Commands.run(climber::spinBackwards, climber));
      // operatorController.x().whileTrue(Commands.run(climber::spinForwards, climber));

        // operatorController.leftBumper().whileTrue(Commands.run(armTest::moveArmUp, armTest));
        // operatorController.rightBumper().whileTrue(Commands.run(armTest::moveArmDown, armTest));
        // operatorController.leftBumper().whileTrue(Commands.run(armTest::runAlgaeIntakeIn, armTest));
        // operatorController.rightBumper().whileTrue(Commands.run(armTest::runAlgaeIntakeOut, armTest));
        // operatorController.leftBumper().whileTrue(Commands.run(armTest::runCoralIntakeIn, armTest));
        // operatorController.rightBumper().whileTrue(Commands.run(armTest::runCoralIntakeOut, armTest));
        // operatorController.leftBumper().whileTrue(Commands.run(armTest::runWristForward, armTest));
        // operatorController.rightBumper().whileTrue(Commands.run(armTest::runWristBackwards, armTest));
      operatorController.b().whileTrue(Commands.run(armTest::stop, armTest));



 
      // elevator.setDefaultCommand(new ManualElevator(
      //   elevator,
      //   () -> getManipLeftY(),
      //   () -> getManipRightY(),
      //   () -> getManiprightBumper(),
      //   () -> getManipleftBumper()
      // )
      // );

      // operatorController.leftBumper().onTrue(new SetElevatorArm(elevator, ArmPosition.Starting));

  }

  public double getManipLeftY(){
    return operatorController.getLeftY();
  }
  public double getManipRightY(){
    return operatorController.getRightY();
  }
  public boolean getManiprightBumper(){
    return operatorController.rightBumper().getAsBoolean();
  }
  public boolean getManipleftBumper(){
    return operatorController.leftBumper().getAsBoolean();
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
  //  */
  public Command getAutonomousCommand()
  {
    // An example command will be run in autonomous
    // return drivebase.getAutonomousCommand("New Auto");
    return Commands.run(armTest::stop, armTest);
  }

  // public void setMotorBrake(boolean brake)
  // {
  //   drivebase.setMotorBrake(brake);
  // }
}