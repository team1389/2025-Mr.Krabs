package frc.command;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.subsystems.ElevatorArm;

public class StartingPos extends SequentialCommandGroup{
    public StartingPos(ElevatorArm elevatorArm){
        addCommands(
            new ParallelCommandGroup(
                new SetShoulder(elevatorArm, .2018),
                new SetWrist(elevatorArm, 62.95), //TODO change
                new SetElevator(elevatorArm, .5177)
            )
        );
    }
}
