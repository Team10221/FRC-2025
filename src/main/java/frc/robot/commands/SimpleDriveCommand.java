package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SwerveSubsystem;

public class SimpleDriveCommand extends Command {
    protected SwerveSubsystem swerve;
    protected DriveCommandType type;
    protected Pose2d targetPose;
    protected ChassisSpeeds targetSpeed;

    public SimpleDriveCommand(SwerveSubsystem sw, Pose2d targetPose, DriveCommandType type) {
        if (type == DriveCommandType.VELOCITY_VECTOR)
            throw new IllegalArgumentException("Wrong constructor. Please pass ChassisSpeeds instead! (LowLevelDriveCommand(SwerveSubsystem sw, ChassisSpeeds sp))");
        this.swerve = sw;
        this.type = type;
        this.targetPose = targetPose;
    }

    public SimpleDriveCommand(SwerveSubsystem sw, ChassisSpeeds sp) {
        this.swerve = sw;
        this.targetSpeed = sp;
        this.type = DriveCommandType.VELOCITY_VECTOR;
    }

    public SwerveSubsystem getSwerveSubsystem() { return swerve; }
    public DriveCommandType getType() { return type; }
    public Pose2d getTargetPose() {
        if (type == DriveCommandType.VELOCITY_VECTOR)
            throw new IllegalStateException("Tried to get the target pose for a linear drive command!");
        return targetPose;
    }
    public ChassisSpeeds getChassisSpeeds() {
        if (type != DriveCommandType.VELOCITY_VECTOR)
            throw new IllegalStateException("Tried to get the velocity vector for a pose drive command!");
        return targetSpeed;
    }

    @Override
    public void end(boolean cancelled) {
        swerve.stop();
    }

    @Override
    public void initialize() {
        switch(type) {
            default -> throw new IllegalArgumentException("??");
            case VELOCITY_VECTOR -> swerve.getSwerveDriveObject().drive(targetSpeed);
            case ROBOT_POSE_RELATIVE -> swerve.drive(targetPose.getTranslation(), targetPose.getRotation(), false);
            case FIELD_POSE_RELATIVE -> swerve.drive(targetPose.getTranslation(), targetPose.getRotation(), true);
        }
    }

    @Override
    public void execute() {
        // no-op
    }
    
    public static enum DriveCommandType {
        VELOCITY_VECTOR,
        ROBOT_POSE_RELATIVE,
        FIELD_POSE_RELATIVE;
    }
}
