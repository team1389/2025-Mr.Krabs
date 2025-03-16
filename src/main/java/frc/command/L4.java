package frc.command;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.subsystems.ElevatorArm;

public class L4 extends SequentialCommandGroup{
    public L4(ElevatorArm elevatorArm){
        addCommands(
            new ParallelCommandGroup(
                // runs l4 sequence
                new SetShoulder(elevatorArm, -12.162),
                new SetElevatorAuto(elevatorArm, 117.5555),
                new SetWrist(elevatorArm, 255)

            )
        );
    }
        
}