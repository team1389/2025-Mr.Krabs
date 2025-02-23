package frc.command;

import java.util.function.Supplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import frc.subsystems.ElevatorArmSubsystem;

public class ManualWrist extends Command{
    ElevatorArmSubsystem elevatorArm;
    Supplier<Double> power;

    public ManualWrist(ElevatorArmSubsystem elevatorArm, Supplier<Double> power){
        this.elevatorArm = elevatorArm;
        this.power = power;
        addRequirements(elevatorArm);
    }

    @Override
    public void execute(){
        elevatorArm.setManualWrist(MathUtil.clamp(power.get(), -1, 1));//}
    }

}
