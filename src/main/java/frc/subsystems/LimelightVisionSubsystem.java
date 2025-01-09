package frc.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;
import frc.util.LimelightHelpers;

//try using robot oritented to find distance
public class LimelightVisionSubsystem extends SubsystemBase{
    NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
    NetworkTableEntry tx1 = table.getEntry("tx");
    NetworkTableEntry ty1 = table.getEntry("ty");
    NetworkTableEntry ta1 = table.getEntry("ta");

    Pose3d blueBotPose = LimelightHelpers.getBotPose3d_wpiBlue("");
    Pose3d botPose = LimelightHelpers.getBotPose3d("");

    Pose2d botPose2d = LimelightHelpers.getBotPose2d("");

    double blueBotXPose = blueBotPose.getX();
    double blueBotYPose = blueBotPose.getY();

    double botXPose = botPose.getX();
    double botYPose = botPose.getY();
    
    double tx = LimelightHelpers.getTX("");
    double ty = LimelightHelpers.getTY("") * (Math.PI/180);
    double ta = LimelightHelpers.getTA(""); //area
    // double dist = (APRILTAGHEIGHT-LIMELIGHTHEIGHT)/ (Math.tan(ty) * Math.cos(tx));

    //measure tx from different angles and make a formula


    public LimelightVisionSubsystem(){
        LimelightHelpers.setLEDMode_PipelineControl("");
        LimelightHelpers.setLEDMode_ForceBlink("");
        LimelightHelpers.setCropWindow("",-1,1,-1,1);
        LimelightHelpers.getTX("");
        
    }

    // public double getAngle(){
    //     return angle;
    // }

    public double getRobotPoseX(){
        return blueBotPose.getX();
    }

    public double getRobotPoseY(){
        return blueBotPose.getY();
    }
    

    public void blink(){
        LimelightHelpers.setLEDMode_ForceBlink("");
    }

    public void on(){
        LimelightHelpers.setLEDMode_ForceOn("");
    }

    public void off(){
        LimelightHelpers.setLEDMode_ForceOff("");
    }
    
    public Pose3d getBotPose3d(){
        return botPose;
    }

    public Pose2d getBotPose2d(){
        return botPose2d;
    }

    @Override
    public void periodic() {
        botPose = LimelightHelpers.getBotPose3d("");
        botXPose = botPose.getX();
        botYPose = botPose.getY();
        botPose2d = LimelightHelpers.getBotPose2d("");

        // SmartDashboard.putNumber("Bot Pose X", botPose.getX());
        // SmartDashboard.putNumber("Bot Pose Y", botPose.getY());
        // SmartDashboard.putNumber("Bot Pose Z", botPose.getZ());
    //    SmartDashboard.putNumber("Bot Pose Rotation", botPose.getRotation());
    }
}
