package frc.subsystems.swervedrive;
import java.util.Optional;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import limelight.Limelight;
import limelight.estimator.PoseEstimate;
import limelight.structures.LimelightResults;
import limelight.structures.target.pipeline.NeuralClassifier;

public class VisionSubsystem extends SubsystemBase{
    Limelight limelight = new Limelight("limelight");
    public double alignTx = 0;


    public VisionSubsystem() {
        limelight.settingsBuilder().withPipelineIndex(0);
    }

    public double getAlignTX(){
        return alignTx;
    }
    @Override
    public void periodic() {
        alignTx = limelight.getNTTable().getEntry("tx").getDouble(0);
    }
}
