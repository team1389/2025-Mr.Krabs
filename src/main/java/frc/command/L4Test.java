package frc.command;

import edu.wpi.first.wpilibj.Timer;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.subsystems.ElevatorArm;

public class L4Test extends SequentialCommandGroup{
    public L4Test(ElevatorArm elevatorArm){

        addCommands(
            new ParallelCommandGroup(
                new SetWrist(elevatorArm, 37.9),
                // new SetWrist(elevatorArm, 37.9),
                new SetElevatorAuto(elevatorArm, 117.5555)
                 
            ),
            new SetShoulderWrist(elevatorArm, -13.162, 270.4)
            // new SetShoulder(elevatorArm, -12.162, 259.4)
            // new ParallelCommandGroup(
                // runs l4 sequence
                
                // new SetWrist(elevatorArm, 270.4), //261.4, 264.4
                // new SetShoulder(elevatorArm, -13.162) //-12.162
                // new SetElevatorAuto(elevatorArm, 117.5555),
                
            // )
        );
    }
        
}
