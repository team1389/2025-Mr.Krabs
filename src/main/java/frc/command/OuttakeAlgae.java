package frc.command;

import edu.wpi.first.wpilibj2.command.Command;
import frc.subsystems.IntakeSubsystem;

public class OuttakeAlgae extends Command {
    public IntakeSubsystem intakesub;

    public OuttakeAlgae(IntakeSubsystem intakesubb) {
        intakesub = intakesubb;
    }

    public void execute() {
        intakesub.spitAlgae();
    }

    public void end(boolean interrupted) {
        intakesub.stopAlgae();
    }
}
