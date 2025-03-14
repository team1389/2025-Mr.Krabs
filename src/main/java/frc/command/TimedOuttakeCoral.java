package frc.command;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.subsystems.IntakeSubsystem;

public class TimedOuttakeCoral extends Command {
    public IntakeSubsystem intakeSub;
    Timer timer = new Timer();

    public TimedOuttakeCoral(IntakeSubsystem intakeSub) {
        this.intakeSub = intakeSub;
        SmartDashboard.putBoolean("IsCoralIn", intakeSub.isCoralIn());
    }

    public void initilize(){
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
    public boolean isFinished() {
        //1 second for now, although it maybe be better to do mmore
        //TODO: fine tune number
        return timer.get() > 1;
    }
}
