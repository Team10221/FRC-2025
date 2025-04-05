package frc.robot.subsystems;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class VisionSubsystem extends SubsystemBase {
    public static record VisionCamera(PhotonCamera camera, PhotonPoseEstimator pose) {}
    protected final VisionCamera[] cameras;
    protected final AprilTagFieldLayout layout;
    protected final SwerveSubsystem sw; 

    public VisionSubsystem(SwerveSubsystem sw) {
        this.sw = sw;
        layout = AprilTagFieldLayout.loadField(Constants.VisionSubsystemConstants.SELECTED_FIELD);
        cameras = new VisionCamera[Constants.VisionSubsystemConstants.CAMERAS.length];
        for (int i = 0; i < cameras.length; i++) {
            var entry = Constants.VisionSubsystemConstants.CAMERAS[i];
            var cam = new PhotonCamera(entry.name());
            cameras[i] = new VisionCamera(
                cam,
                new PhotonPoseEstimator(layout, PoseStrategy.CLOSEST_TO_REFERENCE_POSE, entry.transform())
            );
        }
    }

    // TODO: consider adding data for https://docs.photonvision.org/en/latest/docs/programming/photonlib/robot-pose-estimator.html#additional-photonposeestimator-methods
    @Override
    public void periodic() {
        for (VisionCamera cam : cameras) {
            cam.camera().getAllUnreadResults().forEach(i -> {
                Pose2d previousPose = sw.getRobotPose();
                cam.pose().setReferencePose(previousPose);

                var estimate = cam.pose().update(i);
                if (estimate.isPresent()) {
                    EstimatedRobotPose pose = estimate.get();
                    sw.addVisionMeasurement(pose.estimatedPose.toPose2d(), pose.timestampSeconds);
                }
            });
        }
    }

    public VisionCamera[] getCameras() { return cameras; }
    public AprilTagFieldLayout getLayout() { return layout; }
}
