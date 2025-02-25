package frc.command;

import edu.wpi.first.wpilibj2.command.Command;
import frc.subsystems.ElevatorArm;

public class RunManualShoulder extends Command {
    public ElevatorArm elevatorArm;
    double speed;

    public RunManualShoulder(ElevatorArm elevatorArm, double speed) {
        this.elevatorArm = elevatorArm;
        this.speed = speed;
    }

    @Override
    public void execute() {
        elevatorArm.setManualShoulder(speed);
    }

    @Override
    public void end(boolean interrupted) {
        elevatorArm.setManualShoulder(0);
    }
}
