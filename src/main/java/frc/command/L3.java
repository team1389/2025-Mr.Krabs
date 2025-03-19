package frc.command;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.subsystems.ElevatorArm;

public class L3 extends SequentialCommandGroup{
    public L3(ElevatorArm elevatorArm){
        addCommands(
            new ParallelCommandGroup(
                new SetWrist(elevatorArm, 37.9),
                new SetElevatorAuto(elevatorArm, 117.5555),
                new SetShoulder(elevatorArm, .0057)
            ),
            new ParallelCommandGroup(
                new SetWrist(elevatorArm, 136.5)
            )
        );
    }
}