package frc.command;

import edu.wpi.first.wpilibj2.command.Command;
import frc.subsystems.ClimberSubsystem;

public class runClimberDown extends Command{
    public ClimberSubsystem climberSubsystem;

    public runClimberDown(ClimberSubsystem climberSubsystem1) {
        climberSubsystem = climberSubsystem1;
    }

    public void execute() {
        climberSubsystem.spinBackwards();
    }

    public void end(boolean interrupted) {
        climberSubsystem.stop();
    }
}
