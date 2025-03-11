package frc.command;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.subsystems.ElevatorArm;
import frc.subsystems.IntakeSubsystem;

public class L4Action extends SequentialCommandGroup{
    public L4Action(IntakeSubsystem intake, ElevatorArm elevatorArm){
        addCommands(
            new SetWrist(elevatorArm, 265),
            new ParallelCommandGroup(
                // runs l4 sequence
                new SetShoulder(elevatorArm, -12.162),
                new SetElevator(elevatorArm, 117.5555)
            )
            // new TimedOuttakeCoral(intake)
        );
    }
}