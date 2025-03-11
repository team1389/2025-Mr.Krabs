package frc.command;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.subsystems.ElevatorArm;
import frc.subsystems.IntakeSubsystem;

public class L3 extends SequentialCommandGroup{
    public L3(IntakeSubsystem intake, ElevatorArm elevatorArm){
        addCommands(
            new ParallelCommandGroup(
                new SetShoulder(elevatorArm, .0057),
                new SetWrist(elevatorArm, 265),
                new SetElevator(elevatorArm, 71.4531)
            )
            // new TimedOuttakeCoral(intake)
        );
    }
}
