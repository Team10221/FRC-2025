package frc.robot.auto;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.RobotContainer;

public class AutoFactory {
    public static Command leaveCommand(RobotContainer container) {
        // TODO: tune
        return container.swerve.run(() -> {
            container.swerve.drive(new Translation2d(-1, 0), Rotation2d.kZero, false);
        }).withTimeout(5);
    }

    public static Command followChoreoTrajectory(String name, RobotContainer container) {
        try {
            PathPlannerPath path = PathPlannerPath.fromChoreoTrajectory(name);
            return AutoBuilder.followPath(path).beforeStarting(
                new InstantCommand(() -> 
                    container.swerve.resetPose(
                        path.getStartingHolonomicPose()
                            .orElse(Pose2d.kZero)
                    )
                )
            );
        } catch (Exception e) {
            e.printStackTrace();
            return Commands.none();
        }
    }
}
