package frc.command;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.subsystems.ArmTestSubsystem;
import frc.subsystems.ArmTestSubsystem;

public class IntakeCoralTest extends Command{
    public ArmTestSubsystem intakeSub;

    public IntakeCoralTest(ArmTestSubsystem intakeSub) {
        this.intakeSub = intakeSub;
        addRequirements(intakeSub);
    }

    @Override
    public void execute() {
        intakeSub.runCoralIntakeIn();
        SmartDashboard.putBoolean("IsCoralIn:", intakeSub.getCoralInIntake());
    }

    @Override
    public void end(boolean interrupted) {
        intakeSub.stop();
    }

    // @Override
    // public boolean isFinished(){
    //     return intakeSub.getCoralInIntake();
    // }
}
