package frc.command;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.subsystems.ElevatorArm;
import frc.subsystems.ElevatorArm.ArmPosition;
import frc.subsystems.IntakeSubsystem;

public class L4 extends SequentialCommandGroup{
    public L4(ElevatorArm elevatorArm){
        addCommands(
            new SetElevatorAuto(elevatorArm, 117.5555),
            new ParallelCommandGroup(
                // runs l4 sequence
                // new SetElevatorAuto(elevatorArm, 117.5555)
                new SetWrist(elevatorArm, 255),
                new SetShoulder(elevatorArm, -12.162)


            )
            // new TimedOuttakeCoral(intake)
        );
    }
        
}