package frc.command;

import edu.wpi.first.wpilibj2.command.Command;
import frc.subsystems.ElevatorArm;

public class SetElevatorAuto extends Command{
    public ElevatorArm elevator;
    public double height;

    public SetElevatorAuto(ElevatorArm elevator, double height){
        this.elevator = elevator;
        this.height = height;

        // addRequirements(elevator);
    }

    @Override
    public void execute(){
        elevator.moveToSetpoint(height);
        // elevator.reachGoal(height);
        // elevator.setElevator(height);
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
