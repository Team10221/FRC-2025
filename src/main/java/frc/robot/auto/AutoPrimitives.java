package frc.robot.auto;

import java.io.IOException;
import java.util.concurrent.atomic.AtomicReference;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.FileVersionException;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.CoralManipConstants.CoralManipState;
import frc.robot.Constants.RobotState;
import frc.robot.RobotContainer;
import frc.robot.commands.AlignToTag;

public final class AutoPrimitives {
  private AutoPrimitives() {}

  public static Command timedDrive(RobotContainer c, double x, double y, double rot, double seconds) {
    return c.swerve.run(() -> c.swerve.drive(new Translation2d(x, y), Rotation2d.fromRadians(rot), true)).withTimeout(seconds);
  }

  public static Command goTo(RobotContainer c, double x, double y) {
    return goTo(c, x, y, 0);
  }

  public static Command goTo(RobotContainer c, double x, double y, double rotDeg) {
    PIDController xPid = new PIDController(0.3, 0, 0.1);
    PIDController yPid = new PIDController(0.3, 0, 0.1);
    PIDController rotPid = new PIDController(5, 0, 0.1);
    int MAX_TRANSLATION_SPEED = 2;
    int MAX_ROTATION_SPEED = 3;
    
    xPid.setTolerance(0.05);
    yPid.setTolerance(0.05);
    rotPid.setTolerance(Math.toRadians(2));
    rotPid.enableContinuousInput(-Math.PI, Math.PI);
    
    AtomicReference<Pose2d> targetPose = new AtomicReference<>();
    
    return Commands.sequence(
      new InstantCommand(() -> {
        Pose2d startPose = c.swerve.getRobotPose();
        targetPose.set(new Pose2d(
          startPose.getX() + x,
          startPose.getY() + y,
          Rotation2d.fromDegrees(rotDeg)
        ));
      }),
      c.swerve.run(() -> {
        Pose2d current = c.swerve.getRobotPose();
        Pose2d target = targetPose.get();
        
        double xSpeed = xPid.calculate(current.getX(), target.getX());
        double ySpeed = yPid.calculate(current.getY(), target.getY());
        double rotSpeed = rotPid.calculate(current.getRotation().getRadians(), target.getRotation().getRadians());
        
        xSpeed = Math.max(-MAX_TRANSLATION_SPEED, Math.min(MAX_TRANSLATION_SPEED, xSpeed));
        ySpeed = Math.max(-MAX_TRANSLATION_SPEED, Math.min(MAX_TRANSLATION_SPEED, ySpeed));
        rotSpeed = Math.max(-MAX_ROTATION_SPEED, Math.min(MAX_ROTATION_SPEED, rotSpeed));
        
        c.swerve.drive(new Translation2d(xSpeed, ySpeed), rotSpeed, true);
      }).until(() -> xPid.atSetpoint() && yPid.atSetpoint() && rotPid.atSetpoint()).withTimeout(5.0),
      new InstantCommand(() -> c.swerve.stop())
    );
  }

  public static Command followChoreo(RobotContainer c, String name) {
    try {
      PathPlannerPath path = PathPlannerPath.fromChoreoTrajectory(name);
      return new SequentialCommandGroup(
        new InstantCommand(() -> c.swerve.resetPose(path.getStartingHolonomicPose().orElseThrow())),
        AutoBuilder.followPath(path)
      );
    } catch (IOException | FileVersionException | org.json.simple.parser.ParseException e) {
      e.printStackTrace();
      return Commands.none();
    }
  }

  public static Command coralVision(RobotContainer c, int tagId) {
    return Commands.sequence( // i hate this
      goTo(c, 2.24/2, 0).withTimeout(4), // get halfway there, should take 5s at most (probably way less!)
      new AlignToTag(c.swerve, c.vision, tagId, 1.0, 0.3, 0.3, 0.5)
        .withTimeout(2), // align more
      new InstantCommand(() -> c.stateManager.changeState(RobotState.CORAL_L3)),
      new WaitCommand(0.5), // wait for the shit to go up
      goTo(c, 0, 0.1).withTimeout(1), // TODO PLACEHOLDER must tune `y`
      timedDrive(c, 0.5, 0, 0, 3), // go towards it
      new InstantCommand(() -> c.coralManipulator.setState(CoralManipState.OUTTAKE)),
      new WaitCommand(0.8),
      new InstantCommand(() -> c.coralManipulator.setState(CoralManipState.REST)),
      timedDrive(c, -1, 0, 0, 2),
      new InstantCommand(() -> c.stateManager.changeState(RobotState.DEFAULT))
    );
  }

  public static Command coralBasic(RobotContainer c) { // no idea if this'll work!!!
    return Commands.sequence(
      goTo(c, -(2.24-0.3556)/2, 0),
      new InstantCommand(() -> c.stateManager.changeState(RobotState.CORAL_L3)),
      goTo(c, -(2.24-0.3556), 0),
      new InstantCommand(() -> c.coralManipulator.setState(CoralManipState.OUTTAKE)),
      new WaitCommand(0.8),
      new InstantCommand(() -> c.coralManipulator.setState(CoralManipState.REST)),
      goTo(c, 0.5, 0),
      new InstantCommand(() -> c.stateManager.changeState(RobotState.DEFAULT))
    );
  }

  private static Alliance currentAlliance() {
    return DriverStation.getAlliance().orElse(Alliance.Red);
  }
}
