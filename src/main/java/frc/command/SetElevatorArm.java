package frc.command;

import edu.wpi.first.wpilibj2.command.Command;
import frc.subsystems.ElevatorSubsystem;
import frc.subsystems.ElevatorSubsystem.ArmPosition;

public class SetElevatorArm extends Command{
    private ElevatorSubsystem elevatorArm;
    private ArmPosition target;

    public SetElevatorArm(ElevatorSubsystem elevatorArm, ArmPosition target) {
        this.elevatorArm = elevatorArm;
        this.target = target;

        addRequirements(elevatorArm);
    }

    @Override
    public void execute(){
        elevatorArm.setElevatorArm(target);
    }

    @Override
    public boolean isFinished() {
        return elevatorArm.atTargetPosition();
    }

}
