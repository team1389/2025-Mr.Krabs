package frc.command;

import java.util.Set;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.subsystems.SwerveSubsystem;
import frc.util.TargetingSystem;
import frc.util.TargetingSystem.ReefBranchSide;

public class AlignRightAuto extends SequentialCommandGroup{
    public AlignRightAuto(SwerveSubsystem drivebase, TargetingSystem targetingSystem){
        addCommands(
            targetingSystem.autoTargetCommand(drivebase::getPose)
                .andThen(targetingSystem.setBranchSide(ReefBranchSide.RIGHT))
                .andThen(Commands.runOnce(()->{drivebase.getSwerveDrive().field.getObject("target").setPose(targetingSystem.getCoralTargetPose());}))
                .andThen(Commands.defer(()->drivebase.driveToPose(targetingSystem.getCoralTargetPose()), Set.of(drivebase)))
                );
    }
}
