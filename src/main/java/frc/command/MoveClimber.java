package frc.command;

import edu.wpi.first.wpilibj2.command.Command;
import frc.subsystems.ClimberSubsystem;

public class MoveClimber extends Command{
    public ClimberSubsystem climberSubsystem;
    public double speed;

    public MoveClimber(ClimberSubsystem climberSubsystem1, double speed) {
        this.climberSubsystem = climberSubsystem1;
        this.speed = speed;
    }

    public void execute() {
        climberSubsystem.setSpeed(speed);
    }

    public void end(boolean interrupted) {
        climberSubsystem.stop();
    }
}
