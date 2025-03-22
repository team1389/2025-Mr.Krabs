package frc.command;

import edu.wpi.first.wpilibj.Timer;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.subsystems.ElevatorArm;

public class L4 extends SequentialCommandGroup{
    public L4(ElevatorArm elevatorArm){

        addCommands(
            new ParallelCommandGroup(
                new SetWrist(elevatorArm, 37.9),
                new SetElevatorAuto(elevatorArm, 117.5555)
                 
            ),
            new ParallelCommandGroup(
                // runs l4 sequence
                new SetWrist(elevatorArm, 259.4),
                new SetShoulder(elevatorArm, -12.162)
                // new SetElevatorAuto(elevatorArm, 117.5555),
                
            )
        );
    }
        
}