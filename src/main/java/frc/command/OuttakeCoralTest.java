package frc.command;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.subsystems.ArmTestSubsystem;
import frc.subsystems.ArmTestSubsystem;

public class OuttakeCoralTest extends Command{
    public ArmTestSubsystem intakeSub;

    public OuttakeCoralTest(ArmTestSubsystem intakeSub1) {
        intakeSub = intakeSub1;
    }

    @Override
    public void execute() {
        intakeSub.runCoralIntakeOut();
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
