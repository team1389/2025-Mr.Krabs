package frc.command;

import java.util.Set;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.subsystems.SwerveSubsystem;
import frc.util.TargetingSystem;
import frc.util.TargetingSystem.ReefBranchSide;
import edu.wpi.first.wpilibj.Timer;


public class AlignRightAuto extends Command{
    SwerveSubsystem drivebase;
    TargetingSystem targetingSystem;
    Timer timer = new Timer();
    public AlignRightAuto(SwerveSubsystem drivebase, TargetingSystem targetingSystem){
        this.drivebase = drivebase;
        this.targetingSystem = targetingSystem;
    }

    public void initialize(){
        timer.reset();
        timer.start();
    }

    @Override
    public void execute(){
        targetingSystem.autoTargetCommand(drivebase::getPose)
                .andThen(targetingSystem.setBranchSide(ReefBranchSide.RIGHT))
                .andThen(Commands.runOnce(()->{drivebase.getSwerveDrive().field.getObject("target").setPose(targetingSystem.getCoralTargetPose());}))
                .andThen(Commands.defer(()->drivebase.driveToPose(targetingSystem.getCoralTargetPose()), Set.of(drivebase)));
    }

    @Override
    public boolean isFinished(){
        return timer.get() > 2.5;
    }
}
