package frc.robot.subsystems;

import frc.lib.motor.Motor;
import frc.lib.subsystem.Subsystem;
import frc.robot.Constants.CoralManipConstants.CoralManipAngleState;
import frc.robot.Constants.CoralManipConstants.CoralManipState;
import frc.robot.Constants.AlgaeManipConstants;
import frc.robot.Constants.CoralManipConstants;

public class CoralManipulator extends Subsystem<Double> {
    private final Motor pivotMotor = Motor.neo(CoralManipConstants.ANGLE_MOTOR_ID)
        .setCurrentLimit(CoralManipConstants.PIVOT_CURRENT_LIMIT)
        .setPID(CoralManipConstants.ANGLE_MOTOR_PID)
        .useExternalEncoder();
    private final Motor mechanismMotor = Motor.falcon(CoralManipConstants.MANIP_MOTOR_ID)
        .setCurrentLimit(CoralManipConstants.MANIP_CURRENT_LIMIT);

    public CoralManipulator() {
        super(CoralManipState.class, CoralManipAngleState.class);
    }

    public void updateMotors() {
        mechanismMotor.set(getState(CoralManipState.class).speed);
        pivotMotor.setRef(
            degreesToRotations(
                getState(CoralManipAngleState.class).pos
            )
        );
    }

    public boolean isAtTarget() {
        return pivotMotor.isAtTarget(getState(CoralManipAngleState.class).pos);
    }

    private double degreesToRotations(double degrees) {
        return (degrees / 360) * AlgaeManipConstants.GEAR_RATIO;
    } 

    public void stop() {
        pivotMotor.stop();
        mechanismMotor.stop();
    }
}
