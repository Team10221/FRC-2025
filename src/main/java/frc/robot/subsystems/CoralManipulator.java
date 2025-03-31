package frc.robot.subsystems;

import frc.lib.motor.Motor;
import frc.lib.subsystem.Subsystem;
import frc.robot.Constants.CoralManipConstants.CoralManipAngleState;
import frc.robot.Constants.CoralManipConstants.CoralManipState;
import frc.robot.Constants.CoralManipConstants;

public class CoralManipulator extends Subsystem<Double> {
    private final Motor pivotMotor = Motor.neo(CoralManipConstants.ANGLE_MOTOR_ID)
        .setPID(CoralManipConstants.ANGLE_MOTOR_PID)
        .useExternalEncoder();
    private final Motor mechanismMotor = Motor.falcon(CoralManipConstants.MANIP_MOTOR_ID);

    public CoralManipulator() {
        super(CoralManipState.class, CoralManipAngleState.class);
    }

    public void updateMotors() {
        mechanismMotor.set(getState(CoralManipState.class).speed);
        pivotMotor.setRef(getState(CoralManipAngleState.class).pos);
    }

    public boolean isAtTarget() {
        return pivotMotor.isAtTarget(getState(CoralManipAngleState.class).pos);
    }

    public void stop() {
        pivotMotor.stop();
        mechanismMotor.stop();
    }
}
