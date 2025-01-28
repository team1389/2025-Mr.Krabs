// package frc.subsystems;
// import java.util.Optional;

// import edu.wpi.first.wpilibj2.command.SubsystemBase;
// import limelight.Limelight;
// import limelight.estimator.PoseEstimate;
// import limelight.structures.LimelightResults;
// import limelight.structures.target.pipeline.NeuralClassifier;

// public class VisionSubsystem extends SubsystemBase{
//     Limelight limelight = new Limelight("limelight");

// // Set the limelight to use Pipeline LED control, with the Camera offset of 0, and save.
// limelight.getSetting()
//          .withLimelightLEDMode(LEDMode.PipelineControl)
//          .withCameraOffset(Pose3d.kZero)
//          .save();

// // Get target data
// limelight.getLatestResults().ifPresent((LimelightResults result) -> {
//     for (NeuralClassifier object : result.targets_Classifier)
//     {
//         // Classifier says its a algae.
//         if (object.className.equals("algae"))
//         {
//             // Check pixel location of algae.
//             if (object.ty > 2 && object.ty < 1)
//             {
//               // Algae is valid! do stuff!
//             }
//         }
//     }
// });


// // Required for megatag2 in periodic() function before fetching pose.
// limelight.getSettings()
// 		 .withRobotOrientation(new Orientation3d(gyro.getRotation3d(),
// 												 new AngularVelocity3d(DegreesPerSecond.of(gyro.getPitchVelocity()),
// 																	   DegreesPerSecond.of(gyro.getRollVelocity()),
// 																	   DegreesPerSecond.of(gyro.getYawVelocity()))))
// 		 .save();

// // Get MegaTag2 pose
// Optional<PoseEstimate> visionEstimate = limelight.getPoseEstimator(true).getPoseEstimate();
// // If the pose is present
// visionEstimate.ifPresent((PoseEstimate poseEstimate) -> {
//   // Add it to the pose estimator.
//   poseEstimator.addVisionMeasurement(poseEstimate.pose.toPose2d(), poseEstimate.timestampSeconds);
// });

// // Alternatively you can do
// Optional<PoseEstimate>  BotPose.BLUE_MEGATAG2.get(limelight);
// // If the pose is present
// visionEstimate.ifPresent((PoseEstimate poseEstimate) -> {
//     // Add it to the pose estimator.
//     poseEstimator.addVisionMeasurement(poseEstimate.pose.toPose2d(), poseEstimate.timestampSeconds);
//     });
// }
