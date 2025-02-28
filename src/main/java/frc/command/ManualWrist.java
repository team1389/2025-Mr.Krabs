package frc.command;

import java.util.function.Supplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import frc.subsystems.ElevatorArm;

public class ManualWrist extends Command{
    ElevatorArm elevatorArm;
    Supplier<Double> power;

    public ManualWrist(ElevatorArm elevatorArm, Supplier<Double> power){
        this.elevatorArm = elevatorArm;
        this.power = power;
        addRequirements(elevatorArm);
    }

    @Override
    public void execute(){
        elevatorArm.setManualWrist(MathUtil.clamp(power.get(), -.3, .3));//}
    }

}
