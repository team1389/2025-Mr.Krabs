package frc.command;

import java.util.Set;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.subsystems.SwerveSubsystem;
import frc.util.TargetingSystem;
import frc.util.TargetingSystem.ReefBranchSide;

public class AlignLeftAuto extends SequentialCommandGroup{
    public AlignLeftAuto(SwerveSubsystem drivebase, TargetingSystem targetingSystem){
        addCommands(
            Commands.defer(() -> targetingSystem.autoTargetCommand(drivebase::getPose)
            .andThen(targetingSystem.setBranchSide(ReefBranchSide.LEFT))
            .andThen(Commands.runOnce(()->{drivebase.getSwerveDrive().field.getObject("target").setPose(targetingSystem.getCoralTargetPose());}))
            .andThen(drivebase.driveToPose(targetingSystem.getCoralTargetPose())),
            Set.of(drivebase))
        );
    }
}
