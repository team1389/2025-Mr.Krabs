package frc.command;

import edu.wpi.first.wpilibj2.command.Command;
import frc.subsystems.ElevatorArm;

public class SetElevator extends Command{
    public ElevatorArm elevator;
    public double height;

    public SetElevator(ElevatorArm elevator, double height){
        this.elevator = elevator;
        this.height = height;
    }

    @Override
    public void execute(){
        // elevator.moveToSetpoint(height);
        elevator.setElevator(height);
        // elevator.reachGoal(height);
    }

    @Override
    public boolean isFinished(){
        return elevator.atTargetPosition(height);
    }

    @Override
    public void end(boolean interrupted){
        elevator.setManualElevator(0);
    }
}
