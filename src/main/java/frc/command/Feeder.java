package frc.command;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.subsystems.ElevatorArm;
import frc.subsystems.IntakeSubsystem;

public class Feeder extends SequentialCommandGroup{
    public Feeder(IntakeSubsystem intake, ElevatorArm elevatorArm){
        addCommands(
            new ParallelCommandGroup(
                new SetShoulder(elevatorArm, .19503),
                new SetWrist(elevatorArm, 74.13), //TODO change
                new SetElevator(elevatorArm, .5177)
            )
            // new IntakeCoral(intake)
        );
    }
}
