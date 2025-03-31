package frc.robot.subsystems;

import java.io.File;
import java.io.IOException;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.SwerveDriveConstants;
import swervelib.SwerveDrive;
import swervelib.parser.SwerveParser;

public class SwerveSubsystem extends SubsystemBase {
    protected SwerveDrive swerveDrive;
    protected File swerveJsonDir;
    protected double maximumSpeed;

    public SwerveSubsystem() throws IOException {
        this(SwerveDriveConstants.SWERVE_JSON_DIRECTORY,
                SwerveDriveConstants.MAXIMUM_DRIVETRAIN_SPEED);
    }

    public SwerveSubsystem(File swerveDir, double maxSpeed) throws IOException {
        this.swerveJsonDir = swerveDir;
        this.maximumSpeed = maxSpeed;
        this.swerveDrive = new SwerveParser(swerveDir).createSwerveDrive(maxSpeed);
    }
    public void stop() {
        swerveDrive.drive(new ChassisSpeeds(0D, 0D, 0D));
    }

    public void drive(Translation2d translation, double rotation, boolean fieldRelative) {
        swerveDrive.drive(translation, rotation, fieldRelative, false);
    }

    public void drive(Translation2d translation, Rotation2d rotation, boolean fieldRelative) {
        swerveDrive.drive(translation, rotation.getRadians(), fieldRelative, false);
    }

    public void addVisionMeasurement(Pose2d robotPose, double timestamp) {
        swerveDrive.addVisionMeasurement(robotPose, timestamp);
    }

    public void addVisionMeasurement(Pose2d robotPose, double timestamp, Matrix<N3, N1> stddevs) {
        swerveDrive.addVisionMeasurement(robotPose, timestamp, stddevs);
    }

    public Pose2d getRobotPose() {
        return swerveDrive.getPose();
    }

    public Rotation2d getHeadingData() {
        return swerveDrive.getOdometryHeading();
    }

    public double getMaxSpeed() {
        return maximumSpeed;
    }

    public File getSwerveJsonDir() {
        return swerveJsonDir;
    }

    public SwerveDrive getSwerveDriveObject() {
        return swerveDrive;
    }

    public void zeroGyro() {
        swerveDrive.zeroGyro();
    }

    public void lockPose() {
        swerveDrive.lockPose();
    }

    public void resetOdometry(Pose2d pose) {
        swerveDrive.resetOdometry(pose);
    }
}
