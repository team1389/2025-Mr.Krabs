package frc.command;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.subsystems.ElevatorArm;

public class L2 extends SequentialCommandGroup{
    public L2(ElevatorArm elevatorArm){
        addCommands(
            new ParallelCommandGroup(
                new SetShoulder(elevatorArm, .03976),
                new SetWrist(elevatorArm, 136.5), 
                new SetElevatorAuto(elevatorArm, 24.904855)
            )
        );
    }
}