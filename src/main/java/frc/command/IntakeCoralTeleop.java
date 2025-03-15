package frc.command;

import edu.wpi.first.wpilibj2.command.Command;
import frc.subsystems.IntakeSubsystem;

public class IntakeCoralTeleop extends Command{
    public IntakeSubsystem intakeSub;

    public IntakeCoralTeleop(IntakeSubsystem intakeSub) {
        this.intakeSub = intakeSub;
    }

    @Override
    public void execute() {
        intakeSub.intakeCoral();
    }

    @Override
    public void end(boolean interrupted) {
        intakeSub.stopCoral();
    }

}