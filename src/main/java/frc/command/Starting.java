package frc.command;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.subsystems.ElevatorArm;

public class Starting extends SequentialCommandGroup{
    public Starting(ElevatorArm elevatorArm){
        addCommands(
            new ParallelCommandGroup(
                new SetShoulder(elevatorArm, .2018),
                new SetWrist(elevatorArm, 62.95), //TODO change
                new SetElevatorZero(elevatorArm, .5177)
            )
        );
    }
}