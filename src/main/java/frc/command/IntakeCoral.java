package frc.command;

import edu.wpi.first.wpilibj2.command.Command;
import frc.subsystems.IntakeSubsystem;

public class IntakeCoral extends Command{
    public IntakeSubsystem intakesub;

    public IntakeCoral(IntakeSubsystem intakesubb) {
        intakesub = intakesubb;
    }

    public void execute() {
        intakesub.suckCoral();
    }

    public void end(boolean interrupted) {
        intakesub.stopCoral();
    }
}
