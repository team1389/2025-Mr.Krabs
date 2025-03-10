package frc.command;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.subsystems.ElevatorArm;

public class RunManualWrist extends Command {
    public ElevatorArm elevatorArm;
    double speed;

    public RunManualWrist(ElevatorArm elevatorArm, double speed) {
        this.elevatorArm = elevatorArm;
        this.speed = speed;
        SmartDashboard.putNumber("WristAbsEncoder", elevatorArm.getWristPos());
    }

    @Override
    public void execute() {
        elevatorArm.setManualWrist(speed);
        SmartDashboard.putNumber("WristAbsEncoder", elevatorArm.getWristPos());
    }

    @Override
    public void end(boolean interrupted) {
        elevatorArm.setManualWrist(0);
    }
}
