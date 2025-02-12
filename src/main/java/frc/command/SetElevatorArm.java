package frc.command;

import edu.wpi.first.wpilibj2.command.Command;
import frc.subsystems.ElevatorArmSubsystem;
import frc.subsystems.ElevatorArmSubsystem.ArmPosition;

public class SetElevatorArm extends Command{
    private ElevatorArmSubsystem elevatorArm;
    private ArmPosition target;

    public SetElevatorArm(ElevatorArmSubsystem elevatorArm, ArmPosition target) {
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
