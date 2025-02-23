package frc.command;

import edu.wpi.first.wpilibj2.command.Command;
import frc.subsystems.ElevatorArmSubsystem;

public class SetElevator extends Command{
    public ElevatorArmSubsystem elevator;
    public double height;

    public SetElevator(ElevatorArmSubsystem elevator, double height){
        this.elevator = elevator;
        this.height = height;
    }

    @Override
    public void execute(){
        elevator.setElevator(height);
    }

    @Override
    public boolean isFinished(){
        return elevator.atTargetPosition(height);
    }
}
