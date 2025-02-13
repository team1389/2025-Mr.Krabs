package frc.command;

import edu.wpi.first.wpilibj2.command.Command;
import frc.subsystems.ClimberSubsystem;

public class runClimberUp extends Command{
    public ClimberSubsystem climberSubsystem;

    public runClimberUp(ClimberSubsystem climberSubsystem1) {
        climberSubsystem = climberSubsystem1;
    }

    public void execute() {
        climberSubsystem.spinForwards();
    }

    public void end(boolean interrupted) {
        climberSubsystem.stop();
    }
}
