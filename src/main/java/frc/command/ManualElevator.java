package frc.command;

import java.util.function.Supplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import frc.subsystems.ElevatorArmSubsystem;

public class ManualElevator extends Command{
    ElevatorArmSubsystem elevatorArm;
    Supplier<Double> elevator;

    public ManualElevator(ElevatorArmSubsystem elevatorArm, Supplier<Double> elevator){
        this.elevatorArm = elevatorArm;
        this.elevator = elevator;
        addRequirements(elevatorArm);
    }

    @Override
    public void execute(){
        elevatorArm.setManualElevator(MathUtil.clamp(elevator.get(), -1, 1));
    }


}
