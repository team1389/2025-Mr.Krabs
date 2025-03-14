package frc.command;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.subsystems.ElevatorArm;

public class Feeder extends SequentialCommandGroup{
    public Feeder(ElevatorArm elevatorArm){
        addCommands(
            new ParallelCommandGroup(
                new SetShoulder(elevatorArm, .19503),
                new SetWrist(elevatorArm, 74.13), //TODO change
                new SetElevator(elevatorArm, .5177)
            )
        );
    }
}