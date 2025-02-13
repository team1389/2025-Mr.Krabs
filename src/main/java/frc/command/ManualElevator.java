package frc.command;

import java.util.function.Supplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import frc.subsystems.ElevatorArmSubsystem;

public class ManualElevator extends Command{
    ElevatorArmSubsystem ElevatorArmSubsystem;
    Supplier<Double> elevator, arm1;
    Supplier<Boolean> arm2Up, arm2Down;

    public ManualElevator(ElevatorArmSubsystem ElevatorArmSubsystem, Supplier<Double> elevator, Supplier<Double> arm1, Supplier<Boolean> arm2Up, Supplier<Boolean> arm2Down){
        this.ElevatorArmSubsystem = ElevatorArmSubsystem;
        this.elevator = elevator;
        this.arm1 = arm1;
        this.arm2Up = arm2Up;
        this.arm2Down = arm2Down;
    }

    @Override
    public void execute(){
        ElevatorArmSubsystem.setManualElevator(MathUtil.clamp(elevator.get(), -1, 1));
        ElevatorArmSubsystem.setManualShoulder(MathUtil.clamp(arm1.get(), -1, 1));
        if(arm2Up.get()){
            ElevatorArmSubsystem.setManualElbow(.3);
        } else if(arm2Down.get()){
            ElevatorArmSubsystem.setManualElbow(-.3);
        } else if(!arm2Up.get() && !arm2Down.get()){
            ElevatorArmSubsystem.setManualElbow(0);
        }
    }


}
