package frc.robot.commands;

import java.util.Optional;

import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.VisionSubsystem;

public class AlignToTag extends Command {
  private final SwerveSubsystem sw;
  private final VisionSubsystem vis;
  private final int tagId;
  private final double stopX;
  private final PIDController xPid, yPid, rotPid;

  public AlignToTag(SwerveSubsystem sw, VisionSubsystem vis, int tagId, double stopX, double kPx, double kPy, double kProt) {
    this.sw = sw;
    this.vis = vis;
    this.tagId = tagId;
    this.stopX = stopX;
    this.xPid = new PIDController(kPx, 0, 0);
    this.yPid = new PIDController(kPy, 0, 0);
    this.rotPid = new PIDController(kProt, 0, 0);
    addRequirements(sw);
  }

  @Override
  public void execute() {
    Optional<VisionSubsystem.Target> o = vis.getBestTarget(tagId);
    if (o.isEmpty()) { sw.drive(new Translation2d(), 0, true); return; }
    VisionSubsystem.Target t = o.get();
    Transform3d r2t = t.r2t();
    PhotonTrackedTarget raw = t.raw();

    double x = clamp(xPid.calculate(r2t.getX(), stopX), -1.5, 1.5);
    double y = clamp(yPid.calculate(r2t.getY(), 0.0), -1.0, 1.0);
    double rot = clamp(rotPid.calculate(Math.toRadians(raw.getYaw()), 0.0), -1.5, 1.5);

    sw.drive(new Translation2d(x, y), rot, true);
  }

  @Override
  public void end(boolean interrupted) {
    sw.drive(new Translation2d(), 0, true);
  }

  @Override
  public boolean isFinished() {
    Optional<VisionSubsystem.Target> o = vis.getBestTarget(tagId);
    if (o.isEmpty()) return false;
    Transform3d r2t = o.get().r2t();
    double yaw = Math.abs(o.get().raw().getYaw());
    return Math.abs(r2t.getX() - stopX) < 0.08 && Math.abs(r2t.getY()) < 0.03 && yaw < 2.0;
  }

  private static double clamp(double v, double lo, double hi) {
    return Math.max(lo, Math.min(hi, v));
  }
}
