package frc.command;

import edu.wpi.first.wpilibj2.command.Command;
import frc.subsystems.IntakeSubsystem;

public class OuttakeCoral extends Command {
    public IntakeSubsystem intakeSub;

    public OuttakeCoral(IntakeSubsystem intakeSub) {
        this.intakeSub = intakeSub;
    }

    @Override
    public void execute() {
        intakeSub.outtakeCoral();
    }

    @Override
    public void end(boolean interrupted) {
        intakeSub.stopCoral();
    }
}
