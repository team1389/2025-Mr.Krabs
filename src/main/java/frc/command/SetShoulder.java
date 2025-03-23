package frc.command;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.subsystems.ElevatorArm;

public class SetShoulder extends Command{
    public ElevatorArm shoulder;
    public double shoulderPos, wristPos;

    public SetShoulder(ElevatorArm shoulder, double shoulderPos){
        this.shoulder = shoulder;
        this.shoulderPos = shoulderPos;
        this.wristPos = wristPos;

        // addRequirements(shoulder);
    }

    @Override
    public void initialize(){
        // shoulder.setWristTarget(wristPos);
        shoulder.moveToSetpointShoulder(shoulderPos);
    }

    // @Override
    // public void execute(){
    //     // shoulder.setShoulder(position);
    //     shoulder.moveToSetpointShoulder(position);
    // }

    @Override
    public boolean isFinished(){
        return shoulder.atShoulderTargetPosition(shoulderPos);
    }

    @Override
    public void end(boolean interrupted){
        shoulder.setManualShoulder(0);
    }
}
