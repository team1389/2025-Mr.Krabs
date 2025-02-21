package frc.command;

import java.util.function.Supplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import frc.subsystems.ElevatorArmSubsystem;

public class ManualElevatorUp extends Command{
    ElevatorArmSubsystem ElevatorArmSubsystem;

    public ManualElevatorUp(ElevatorArmSubsystem ElevatorArmSubsystem){
        this.ElevatorArmSubsystem = ElevatorArmSubsystem;
    }

    @Override
    public void execute(){
        ElevatorArmSubsystem.setManualElevatorUp();
    }

    @Override
    public void end(boolean interrupted) {
        ElevatorArmSubsystem.stop();
    }


}
