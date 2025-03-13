package frc.command;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.subsystems.ElevatorArm;
import frc.subsystems.ElevatorArm.ArmPosition;
import frc.subsystems.IntakeSubsystem;

public class L4Action extends Command{
    private ElevatorArm elevatorArm;
    private ArmPosition pos;
    private double elevatorPos, shoulderPos, wristPos;
    public L4Action(ElevatorArm elevatorArm, ArmPosition pos){
        this.elevatorArm = elevatorArm;
        this.pos = pos;
    }

    public void initilize(){
        elevatorPos = elevatorArm.returnElevatorTarget(pos);
        shoulderPos = elevatorArm.returnShoulderTarget(pos);
        wristPos = elevatorArm.returnWristTarget(pos);
    }

    @Override
    public void execute(){
        // if(!elevatorArm.atTargetPosition(117.5555)){
            elevatorArm.setElevator(117.5555);
        // } else {
        //     elevatorArm.setManualElevator(0);
        // }

        // if(!elevatorArm.atShoulderTargetPosition(-12.162)){
            elevatorArm.moveToSetpointShoulder(-12.162);
        // } else{
        //     elevatorArm.setManualShoulder(0);
        // }

        // if(!elevatorArm.atWristTargetPosition(265)){
            elevatorArm.setWrist(265);
        // } else{
        //     elevatorArm.setManualWrist(0);
        // }

    }

    @Override
    public boolean isFinished(){
        return elevatorArm.atTargetPosition(elevatorPos);
                
    }

        // addCommands(
        //     new SetWrist(elevatorArm, 265),
        //     new ParallelCommandGroup(
        //         // runs l4 sequence
        //         new SetShoulder(elevatorArm, -12.162),
        //         new SetElevator(elevatorArm, 117.5555)
        //     )
        //     // new TimedOuttakeCoral(intake)
        // );
}