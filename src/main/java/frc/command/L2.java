package frc.command;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.subsystems.ElevatorArm;
import frc.subsystems.IntakeSubsystem;

public class L2 extends SequentialCommandGroup{
    public L2(IntakeSubsystem intake, ElevatorArm elevatorArm){
        addCommands(
            new ParallelCommandGroup(
                new SetShoulder(elevatorArm, .03976),
                new SetWrist(elevatorArm, 265), //TODO change
                new SetElevator(elevatorArm, 32.9788)
            )
            // new TimedOuttakeCoral(intake)
        );
    }
}
