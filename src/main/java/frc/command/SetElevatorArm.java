package frc.command;

import edu.wpi.first.wpilibj2.command.Command;
import frc.subsystems.ElevatorArm.ArmPosition;
import frc.subsystems.ElevatorArm;

public class SetElevatorArm extends Command{
    private ElevatorArm elevatorArm;
    private ArmPosition target;
    double height, shoulder, wrist;

    public SetElevatorArm(ElevatorArm elevatorArm, ArmPosition target, double height, double shoulder, double wrist) {
        this.elevatorArm = elevatorArm;
        this.target = target;
        this.height = height;

        addRequirements(elevatorArm);
    }

    public void initilize(){
        elevatorArm.setElevatorArm(target);
    }

    @Override
    public void execute(){
        // elevatorArm.moveElevatorArm();
        elevatorArm.setElevator(height);
        elevatorArm.moveToSetpointShoulder(shoulder);
        // elevatorArm.setWrist(wrist);
    }

    @Override
    public boolean isFinished() {
        return elevatorArm.atTargetPosition();
    }

    @Override
    public void end(boolean interrupted){
        elevatorArm.stop();
        // elevatorArm.holdPosition();
    }

}
