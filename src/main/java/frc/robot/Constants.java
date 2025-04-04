// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.File;

import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Filesystem;
import frc.lib.util.PID;

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
        // transform = transformation from robot center to camera location
        public static record CameraConfiguration(String name, Transform3d transform) {}
        /**
         * OI hi there, there's both an AndyMark and an welded field for this year
         * - use AndyMark if you're on a public field (and they use andymark themselves)
         * - use welded if you're on an actual frc comp
         */
        public static AprilTagFields SELECTED_FIELD = AprilTagFields.kDefaultField; // "k2025ReefscapeWelded"
        public static final CameraConfiguration[] CAMERAS = {
            new CameraConfiguration("FRONT", null) // (main) camera, probably lol
        };
    }

    public final class SwerveDriveConstants {
        // yo uh i pulled this from https://www.chiefdelphi.com/t/yagsl-working-configurations/492586/9 btw
        // (aka https://github.com/frc457/EverybotSwerveYAGSL/tree/main/src/main/deploy/swerve/maxSwerve/modules)
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

        public static final PID ANGLE_MOTOR_PID = new PID(0, 0, 0);

        public static enum AlgaeManipAngleState {
            UP(0), OUT(0); // TODO: replace w/ actual values

            public final double position;

            AlgaeManipAngleState(double position) {
                this.position = position;
            }
        }

        public static enum AlgaeManipState {
            IDLE(0, 0), INTAKE(1, -1), RELEASE(-0.5, 0.5);

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

        public static final PID ANGLE_MOTOR_PID = new PID(0, 0, 0);

        public static enum CoralManipAngleState {
            IDLE(0), INTAKE(0), SCORING(0); // tbd thru testing

            public final double pos;
            CoralManipAngleState(double pos) {
                this.pos = pos;
            }
        }

        public static enum CoralManipState {
            REST(0), INTAKE(0), OUTTAKE(0), HOLD(0);

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

        public static final double MAX_HEIGHT = 0.7366; // in meters
        public static final double SPROCKET_CIRCUMFERENCE = 0.1397;
        public static final double GEAR_RATIO = 9;
        public static final double TOLERANCE = 0.02;
        
        public static final PID ELEVATOR_PID = new PID(0, 0, 0, 0);

        public static enum ElevatorState {
            DOWN(0), L1(0), L2(0), L3(0), L4(0),
            ALG1(0), ALG2(0), MID(MAX_HEIGHT * 0.5), MAX(MAX_HEIGHT);

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

    public final class ControllerConstants {
        public static final int PRIMARY_PORT = 0;
        public static final int SECONDARY_PORT = 1;
        public static final double DEADBAND = 0.05;
        public static final double ROTATION_SPEED = 2;
    }
}
