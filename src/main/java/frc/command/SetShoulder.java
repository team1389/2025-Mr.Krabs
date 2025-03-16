package frc.command;

import edu.wpi.first.wpilibj2.command.Command;
import frc.subsystems.ElevatorArm;

public class SetShoulder extends Command{
    public ElevatorArm shoulder;
    public double position;

    public SetShoulder(ElevatorArm shoulder, double position){
        this.shoulder = shoulder;
        this.position = position;
    }

    @Override
    public void execute(){
        shoulder.moveToSetpointShoulder(position);
    }

    @Override
    public boolean isFinished(){
        return shoulder.atShoulderTargetPosition(position);
    }

    @Override
    public void end(boolean interrupted){
        shoulder.setManualShoulder(0);
    }
}
