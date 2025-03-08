package frc.command;

import edu.wpi.first.wpilibj2.command.Command;
import frc.subsystems.ElevatorArm.ArmPosition;
import frc.subsystems.ElevatorArm;

public class SetElevatorArm extends Command{
    private ElevatorArm elevatorArm;
    private ArmPosition target;

    public SetElevatorArm(ElevatorArm elevatorArm, ArmPosition target) {
        this.elevatorArm = elevatorArm;
        this.target = target;

        addRequirements(elevatorArm);
    }

    public void initilize(){
        elevatorArm.setElevatorArm(target);
    }

    @Override
    public void execute(){
        elevatorArm.moveElevatorArm();
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
