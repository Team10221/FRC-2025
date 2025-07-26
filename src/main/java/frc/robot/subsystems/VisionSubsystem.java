package frc.robot.subsystems;

import java.util.ArrayList;
import java.util.Comparator;
import java.util.List;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class VisionSubsystem extends SubsystemBase {
  public static record CameraConfiguration(String name, Transform3d r2c) {}
  public static record VisionCamera(PhotonCamera cam, PhotonPoseEstimator est, Transform3d r2c) {}
  public static record Target(VisionCamera vc, PhotonPipelineResult res, PhotonTrackedTarget raw, Transform3d r2t) {}

  private final VisionCamera[] cams;
  private final AprilTagFieldLayout layout;
  private final SwerveSubsystem sw;

  public static final double AMBIGUITY_MAX = 0.2;
  public static final double JUMP_REJECT_METERS = 3.0;

  public VisionSubsystem(SwerveSubsystem sw) {
    this.sw = sw;
    this.layout = AprilTagFieldLayout.loadField(Constants.VisionSubsystemConstants.SELECTED_FIELD);
    this.cams = new VisionCamera[Constants.VisionSubsystemConstants.CAMERAS.length];
    for (int i = 0; i < cams.length; i++) {
      var cfg = Constants.VisionSubsystemConstants.CAMERAS[i];
      var cam = new PhotonCamera(cfg.name());
      var est = new PhotonPoseEstimator(layout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, cfg.transform());
      est.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
      cams[i] = new VisionCamera(cam, est, cfg.transform());
    }
  }

  @Override
  public void periodic() {
    for (VisionCamera vc : cams) {
      var list = vc.cam().getAllUnreadResults();
      for (PhotonPipelineResult r : list) {
        vc.est().setReferencePose(sw.getRobotPose());
        Optional<EstimatedRobotPose> o = vc.est().update(r);
        if (o.isEmpty()) continue;
        EstimatedRobotPose p = o.get();
        if (!r.hasTargets()) continue;
        double amb = r.getBestTarget().getPoseAmbiguity();
        if (amb > AMBIGUITY_MAX) continue;
        Pose2d cur = sw.getRobotPose();
        if (cur.getTranslation().getDistance(p.estimatedPose.toPose2d().getTranslation()) > JUMP_REJECT_METERS) continue;
        sw.addVisionMeasurement(p.estimatedPose.toPose2d(), p.timestampSeconds);
      }
    }
  }

  public Optional<Target> getBestTarget(int fid) {
    List<Target> all = new ArrayList<>();
    for (VisionCamera vc : cams) {
      var r = vc.cam().getLatestResult();
      if (!r.hasTargets()) continue;
      for (PhotonTrackedTarget t : r.getTargets()) {
        if (fid > 0 && t.getFiducialId() != fid) continue;
        Transform3d c2t = t.getBestCameraToTarget();
        Transform3d r2t = vc.r2c().plus(c2t);
        all.add(new Target(vc, r, t, r2t));
      }
    }
    if (all.isEmpty()) return Optional.empty();
    all.sort(Comparator
      .comparingDouble((Target a) -> a.raw().getPoseAmbiguity())
      .thenComparingDouble(a -> Math.abs(a.raw().getYaw()))
      .thenComparingDouble(a -> a.r2t().getTranslation().getNorm()));
    return Optional.of(all.get(0));
  }

  public VisionCamera[] getCameras() { return cams; }
  public AprilTagFieldLayout getLayout() { return layout; }
}
