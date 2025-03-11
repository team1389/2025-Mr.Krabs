package frc.command;

import java.util.function.Supplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.subsystems.ElevatorArm;

public class ManualElevatorArm extends Command{
    ElevatorArm elevatorArm;
    Supplier<Double> armPower, elevatorPower;

    public ManualElevatorArm(ElevatorArm elevatorArm, Supplier<Double> armPower, Supplier<Double> elevatorPower){
        this.elevatorArm = elevatorArm;
        this.armPower = armPower;
        this.elevatorPower = elevatorPower;
        addRequirements(elevatorArm);
    }

    @Override
    public void execute(){
        // if((elevatorArm.ifElevatorTooHigh() && (MathUtil.clamp(elevatorPower.get(), -1, 1) > 0)) ||
        // (elevatorArm.ifElevatorTooLow() && (MathUtil.clamp(elevatorPower.get(), -1, 1) < 0))){
            elevatorArm.setManualElevator(MathUtil.clamp(elevatorPower.get(), -1, 1));
        // }
        elevatorArm.setManualWrist(MathUtil.clamp(armPower.get(), -1, 1));
    }

}
