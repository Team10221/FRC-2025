// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.util.Units;
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
    public final class Swerve {
        public static final double maximumSpeed = Units.feetToMeters(14.63);
    }

    public final class AlgaeManipulatorConstants {
        public static final int LEFT_MOTOR_ID = 9;
        public static final int RIGHT_MOTOR_ID = 10;
        public static final int ANGLE_MOTOR_ID = 11;

        public static final PID ANGLE_MOTOR_PID = new PID(0, 0, 0);

        public static enum AngleState {
            IDLE(0), UP(0); // TODO: replace w/ actual values

            public final double position;

            AngleState(double position) {
                this.position = position;
            }
        }

        public static enum ManipulatorState {
            INTAKE(1, -1), RELEASE(-0.5, 0.5); // TODO: replace w/ actual values

            public final double leftMotorSpeed, rightMotorSpeed;

            ManipulatorState(double leftMotorSpeed, double rightMotorSpeed) {
                this.leftMotorSpeed = leftMotorSpeed;
                this.rightMotorSpeed = rightMotorSpeed;
            }
        }
    }
}
