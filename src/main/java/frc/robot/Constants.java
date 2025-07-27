// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.File;
import java.util.List;

import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Filesystem;
import frc.lib.util.PID;
import frc.robot.Constants.AlgaeManipConstants.AlgaeManipAngleState;
import frc.robot.Constants.CoralManipConstants.CoralManipAngleState;
import frc.robot.Constants.ElevatorConstants.ElevatorState;
import frc.robot.subsystems.VisionSubsystem.CameraConfiguration;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    public final class VisionSubsystemConstants {
        public static record CameraConfiguration(String name, Transform3d transform) {}
        public static AprilTagFields SELECTED_FIELD = AprilTagFields.k2025ReefscapeWelded;
        public static final CameraConfiguration[] CAMERAS = {
                new CameraConfiguration("Arducam_OV9281_USB_Camera (1)", new Transform3d(new Translation3d(26.935 * 100, -10.824 * 100, (40.630 + 2.858) * 100), new Rotation3d())) // (main) camera, probably lol
        };
    }

    public final class SwerveDriveConstants {
        // yo uh i pulled this from
        // https://www.chiefdelphi.com/t/yagsl-working-configurations/492586/9 btw
        // (aka
        // https://github.com/frc457/EverybotSwerveYAGSL/tree/main/src/main/deploy/swerve/maxSwerve/modules)
        public static final File SWERVE_JSON_DIRECTORY = new File(Filesystem.getDeployDirectory(), "swerve/maxSwerve");
        public static final double MAXIMUM_DRIVETRAIN_SPEED = Units.feetToMeters(14.63);
        // SHEESH
        // w work eddie
    }

    public final class AlgaeManipConstants {
        public static final int LEFT_MOTOR_ID = 11;
        public static final int RIGHT_MOTOR_ID = 12;
        public static final int ANGLE_MOTOR_ID = 13;

        public static final int END_MOTORS_CURRENT_LIMIT = 20;
        public static final int PIVOT_CURRENT_LIMIT = 40;

        public static final double GEAR_RATIO = 45;

        public static final PID ANGLE_MOTOR_PID = new PID(0.05, 0, 0.01);

        public static enum AlgaeManipAngleState {
            UP(-10), STOW(-70), OUT(-135);

            public final double position;

            AlgaeManipAngleState(double position) {
                this.position = position;
            }
        }

        public static enum AlgaeManipState {
            IDLE(0, 0), INTAKE(0.6, -0.6), RELEASE(-0.5, 0.5);

            public final double leftMotorSpeed, rightMotorSpeed;

            AlgaeManipState(double leftMotorSpeed, double rightMotorSpeed) {
                this.leftMotorSpeed = leftMotorSpeed;
                this.rightMotorSpeed = rightMotorSpeed;
            }
        }
    }

    public final class CoralManipConstants {
        public static final int MANIP_MOTOR_ID = 14;
        public static final int ANGLE_MOTOR_ID = 15;

        public static final int MANIP_CURRENT_LIMIT = 30;
        public static final int PIVOT_CURRENT_LIMIT = 40;

        public static final double GEAR_RATIO = 45;

        public static final PID ANGLE_MOTOR_PID = new PID(0.04, 0, 0.001);

        public static enum CoralManipAngleState {
            INTAKE(0), SCORE(-165); // -155 probably works too

            public final double pos;

            CoralManipAngleState(double pos) {
                this.pos = pos;
            }
        }

        public static enum CoralManipState {
            REST(0), INTAKE(-0.5), OUTTAKE(0.7);

            public final double speed;

            CoralManipState(double speed) {
                this.speed = speed;
            }
        }
    }

    public final class ElevatorConstants {
        public static final int LEADER_MOTOR_ID = 9;
        public static final int FOLLOWER_MOTOR_ID = 10;
        public static final int CURRENT_LIMIT = 40;

        // unsure if this value is correct - should be trivial to adjust
        public static final double MAX_HEIGHT = 0.7366; // in meters

        public static final double SPROCKET_CIRCUMFERENCE = 0.1397;
        public static final double GEAR_RATIO = 9;
        public static final double TOLERANCE = 0.02;

        public static final PID ELEVATOR_PID = new PID(0.3, 0, 0);

        public static enum ElevatorState {
            DOWN(0), 
            ALGAE_L2(MAX_HEIGHT * 0.46206896551),
            ALGAE_L3(MAX_HEIGHT * 0.7448275862),
            ALGAE_PROCESSOR(0),
            CORAL_L1(MAX_HEIGHT * 0.2),
            CORAL_L2(MAX_HEIGHT * 0.24), 
            CORAL_L3(MAX_HEIGHT * 0.53),
            CORAL_L4(MAX_HEIGHT * 0.97), // todo adjust 
            CORAL_TRANSITION(MAX_HEIGHT * 0.2),
            TEST(MAX_HEIGHT * 0.85);

            public final double height;

            ElevatorState(double height) {
                this.height = height;
            }
        }
    }

    public final class MotorTestConstants {
        public static final double elevatorSpeed = 0.6;

        public static enum MotorTestState {
            REST(0), FORWARD(elevatorSpeed), REVERSE(-elevatorSpeed);

            public final double speed;

            MotorTestState(double speed) {
                this.speed = speed;
            }
        }
    }

    public static enum RobotState {
        DEFAULT(
                ElevatorState.DOWN,
                AlgaeManipAngleState.UP,
                CoralManipAngleState.INTAKE,
                1),
        CORAL_TRANSITION_IN(
                ElevatorState.CORAL_TRANSITION,
                AlgaeManipAngleState.UP,
                CoralManipAngleState.INTAKE,
                1),
        CORAL_TRANSITION_OUT(
                ElevatorState.CORAL_TRANSITION,
                AlgaeManipAngleState.UP,
                CoralManipAngleState.SCORE,
                1),
        CORAL_L1(
                ElevatorState.CORAL_L1,
                AlgaeManipAngleState.UP,
                CoralManipAngleState.SCORE,
                0.8),
        CORAL_L2(
                ElevatorState.CORAL_L2,
                AlgaeManipAngleState.UP,
                CoralManipAngleState.SCORE,
                0.6),
        CORAL_L3(
                ElevatorState.CORAL_L3,
                AlgaeManipAngleState.UP,
                CoralManipAngleState.SCORE,
                0.4),
        CORAL_L4(
                ElevatorState.CORAL_L4,
                AlgaeManipAngleState.UP,
                CoralManipAngleState.SCORE,
                0.2),
        ALGAE_REEF_INTAKE_L2(
                ElevatorState.ALGAE_L2,
                AlgaeManipAngleState.OUT,
                CoralManipAngleState.SCORE,
                0.6),
        ALGAE_REEF_INTAKE_L3(
                ElevatorState.ALGAE_L3,
                AlgaeManipAngleState.OUT,
                CoralManipAngleState.SCORE,
                0.6),
        ALGAE_OUTTAKE_PROCESSOR(
                ElevatorState.ALGAE_PROCESSOR,
                AlgaeManipAngleState.OUT,
                CoralManipAngleState.INTAKE,
                1);

        public ElevatorState elevatorState;
        public AlgaeManipAngleState algaeManipAngleState;
        public CoralManipAngleState coralManipAngleState;
        public double driveSpeedMultiplier;

        RobotState(
                ElevatorState elevatorState,
                AlgaeManipAngleState algaeManipAngleState,
                CoralManipAngleState coralManipAngleState,
                double driveSpeedMultiplier) {
            this.elevatorState = elevatorState;
            this.algaeManipAngleState = algaeManipAngleState;
            this.coralManipAngleState = coralManipAngleState;
            this.driveSpeedMultiplier = driveSpeedMultiplier;
        }
    }

    public final class ControllerConstants {
        public static final int PRIMARY_PORT = 0;
        public static final int SECONDARY_PORT = 1;
        public static final double DEADBAND = 0.05;
        public static final double ROTATION_SPEED = 2;
    }
}
