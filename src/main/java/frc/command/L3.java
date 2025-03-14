package frc.command;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.subsystems.ElevatorArm;
import frc.subsystems.IntakeSubsystem;

public class L3 extends SequentialCommandGroup{
    public L3(ElevatorArm elevatorArm){
        addCommands(
            new ParallelCommandGroup(
                new SetShoulder(elevatorArm, .0057),
                new SetWrist(elevatorArm, 79.09802),
                new SetElevator(elevatorArm, 71.4531)
            )
        );
    }
}
