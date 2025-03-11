package frc.command;

import edu.wpi.first.wpilibj2.command.Command;
import frc.subsystems.ElevatorArm;

public class SetWrist extends Command{
    public ElevatorArm wrist;
    public double position;

    public SetWrist(ElevatorArm wrist, double position){
        this.wrist = wrist;
        this.position = position;
    }

    @Override
    public void execute(){
        wrist.moveToSetpointWrist(position);
        // wrist.setWrist(position);
    }

    @Override
    public boolean isFinished(){
        return wrist.atWristTargetPosition(position);
    }

    @Override
    public void end(boolean interrupted){
        wrist.setManualWrist(0);
    }
}
