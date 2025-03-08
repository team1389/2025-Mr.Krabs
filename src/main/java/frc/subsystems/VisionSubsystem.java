package frc.subsystems;
import java.util.Optional;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import limelight.Limelight;
// import limelight.estimator.PoseEstimate;
// import limelight.structures.LimelightResults;
// import limelight.structures.target.pipeline.NeuralClassifier;

public class VisionSubsystem extends SubsystemBase{
    private final AprilTagFieldLayout aprilTagFieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeAndyMark);
    Limelight limelight = new Limelight("limelight-front");
    public double alignTx = 0;


    public VisionSubsystem() {
    }

    public double getAlignTX(){
        return alignTx;
    }
    @Override
    public void periodic() {
        alignTx = limelight.getNTTable().getEntry("tx").getDouble(0);
    }
}
