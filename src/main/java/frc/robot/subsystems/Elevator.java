package frc.robot.subsystems;

import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;

import frc.lib.motor.Motor;
import frc.lib.subsystem.Subsystem;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.ElevatorConstants.ElevatorState;

public class Elevator extends Subsystem<Double> {
    private final Motor leaderMotor = Motor.falcon(ElevatorConstants.LEADER_MOTOR_ID)
        .setPID(ElevatorConstants.ELEVATOR_PID)
        .setCurrentLimit(ElevatorConstants.CURRENT_LIMIT);
    private final Motor followerMotor = Motor.falcon(ElevatorConstants.FOLLOWER_MOTOR_ID)
        .setCurrentLimit(ElevatorConstants.CURRENT_LIMIT)
        .config(adapter -> {
            ((TalonFX) adapter.getMotorController())
                .setControl(
                    new Follower(
                        ElevatorConstants.LEADER_MOTOR_ID, 
                        true
                    )
                );
        });

    public Elevator() {
        super(ElevatorState.class);
        leaderMotor.config(adapter -> {
            double maxRotations = heightToRotations(ElevatorConstants.MAX_HEIGHT);
            adapter.setSoftLimits(maxRotations, 0);
        });
    }

    public void updateMotors() {
        leaderMotor.setRef(
            heightToRotations(
                getState(ElevatorState.class).height
            )
        );
    }

    // might have to update these methods in accordance with the nature of a 2 stage elevator
    // TODO - must test though

    public double heightToRotations(double heightMeters) {
        if (heightMeters < 0 || heightMeters > ElevatorConstants.MAX_HEIGHT) {
            throw new IllegalArgumentException("Invalid height: " + heightMeters);
        }
        return (heightMeters / ElevatorConstants.SPROCKET_CIRCUMFERENCE) * ElevatorConstants.GEAR_RATIO;
    }

    public double rotationsToHeight(double rotations) {
        return (rotations / ElevatorConstants.GEAR_RATIO) * ElevatorConstants.SPROCKET_CIRCUMFERENCE;
    }

    public boolean isAtTarget() {
        double currentHeight = rotationsToHeight(leaderMotor.getPosition());
        double targetHeight = getState(ElevatorState.class).height;
        return Math.abs(currentHeight - targetHeight) < ElevatorConstants.TOLERANCE;
    }

    public void stop() {
        leaderMotor.stop();
        followerMotor.stop();
    }
}
