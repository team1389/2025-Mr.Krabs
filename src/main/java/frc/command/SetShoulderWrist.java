package frc.command;

import edu.wpi.first.wpilibj2.command.Command;
import frc.subsystems.ElevatorArm;

public class SetShoulderWrist extends Command{
    public ElevatorArm arm;
    private double shoulderTarget, wristTarget;

    public SetShoulderWrist(ElevatorArm arm, double shoulderTarget, double wristTarget){
        this.arm = arm;
        this.shoulderTarget = shoulderTarget;
        this.wristTarget = wristTarget;

        // addRequirements(arm);
    }

    @Override
    public void initialize(){
        arm.setShoulderTarget(shoulderTarget);
        arm.setWristTarget(wristTarget);
    }

    @Override
    public void execute(){
        arm.updateShoulder();
        arm.updateWrist();
    }

    @Override
    public boolean isFinished(){
        return arm.ifWristAtTarget() && arm.ifShoulderAtTarget();
    }

    @Override
    public void end(boolean interrupted){
        arm.stop();
    }
}
