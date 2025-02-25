package frc.command;

import edu.wpi.first.wpilibj2.command.Command;
import frc.subsystems.IntakeSubsystem;

public class IntakeCoral extends Command{
    public IntakeSubsystem intakeSub;

    public IntakeCoral(IntakeSubsystem intakeSub) {
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

    // @Override
    // public boolean isFinished(){
    //     return intakeSub.ifCoral();
    // }
}
