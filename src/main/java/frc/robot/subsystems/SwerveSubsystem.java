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
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

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
    
        double moduleOffset = Units.inchesToMeters(12.25);

        ModuleConfig moduleConfig = new ModuleConfig(
            0.038, 
            SwerveDriveConstants.MAXIMUM_DRIVETRAIN_SPEED, 
            0.7, 
            DCMotor.getNEO(1), 
            5.08, 
            40.0, 
            1);
        RobotConfig config = new RobotConfig(
            100, 
            7, 
            moduleConfig, 
            new Translation2d(moduleOffset, moduleOffset), // FL
            new Translation2d(moduleOffset, -moduleOffset), // FR
            new Translation2d(-moduleOffset, moduleOffset), // BL
            new Translation2d(-moduleOffset, -moduleOffset)); // BR
        // Configure AutoBuilder last    
        AutoBuilder.configure(
            this::getPose, // Robot pose supplier
            this::resetPose, // Method to reset odometry (will be called if your auto has a starting pose)
            this::getRobotRelativeSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
            (speeds, feedforwards) -> this.driveRobotRelative(speeds), // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds. Also optionally outputs individual module feedforwards
            new PPHolonomicDriveController( // PPHolonomicController is the built in path following controller for holonomic drive trains
                    new PIDConstants(5.0, 0.0, 0.0), // Translation PID constants
                    new PIDConstants(5.0, 0.0, 0.0) // Rotation PID constants
            ),
            config, // The robot configuration
            () -> {
            // Boolean supplier that controls when the path will be mirrored for the red alliance
            // This will flip the path being followed to the red side of the field.
            // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

            var alliance = DriverStation.getAlliance();
            if (alliance.isPresent()) {
                return alliance.get() == DriverStation.Alliance.Red;
            }
            return false;
            },
            this // Reference to this subsystem to set requirements
        );
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

    // methods for PathPlannerLib
    private ChassisSpeeds getRobotRelativeSpeeds() {
        return swerveDrive.getRobotVelocity();
    }

    public void resetPose(Pose2d pose2d) {
        swerveDrive.resetOdometry(pose2d);
    }

    private Pose2d getPose() {
        return swerveDrive.getPose();
    }

    private void driveRobotRelative(ChassisSpeeds speeds) {
        swerveDrive.drive(speeds);
    }
}
