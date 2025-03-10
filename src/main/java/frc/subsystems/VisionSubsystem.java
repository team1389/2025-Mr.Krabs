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
    public double targetID = 0;
    double[] pos = new double[2];
    //pick the default to be whatever is on our wood feild


    public VisionSubsystem() {
    }

    public double getTargetID(){
        return targetID;
    }

    public double getAlignTX(){
        return alignTx;
    }
    
    public double[] getRobotPosition(){
        //last value is z which is usless just dont touch it since it works. just ignore pos[2]
        return pos;
    }

    public boolean canLimelightSeeTag(){
        return limelight.getNTTable().getEntry("tv").getDouble(0) == 1;
    }
    //botpose_orb_wpiblue
    //botpose_orb_wpired
    @Override
    public void periodic() {
        //TODO: test if wpiblue works instead of botpose
        pos = limelight.getNTTable().getEntry("botpose_wpiblue").getDoubleArray(new double[]{0,0,0});
        alignTx = limelight.getNTTable().getEntry("tx").getDouble(0);
        //TODO: check if it works with double. May have to be integer
        targetID = limelight.getNTTable().getEntry("tid").getDouble(0);
    }
}
