package frc.command;

import edu.wpi.first.wpilibj.Timer;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.subsystems.IntakeSubsystem;

public class OuttakeCoral extends Command {
    public IntakeSubsystem intakeSub;
    public Timer timer;

    public OuttakeCoral(IntakeSubsystem intakeSub) {
        this.intakeSub = intakeSub;
        timer = new Timer();
        SmartDashboard.putBoolean("IsCoralIn", intakeSub.isCoralIn());
    }

    public void initialize(){
        timer.reset();
        timer.start();
    }

    @Override
    public void execute() {
        intakeSub.outtakeCoral();
        SmartDashboard.putBoolean("IsCoralIn", intakeSub.isCoralIn());
    }

    @Override
    public void end(boolean interrupted) {
        intakeSub.stopCoral();
    }

    @Override
    public boolean isFinished(){
        return timer.get() > .5;
    }
}
