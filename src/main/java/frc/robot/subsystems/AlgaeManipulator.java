package frc.robot.subsystems;

import frc.lib.motor.Motor;
import frc.lib.subsystem.Subsystem;
import frc.robot.Constants.AlgaeManipConstants;
import frc.robot.Constants.AlgaeManipConstants.AlgaeManipState;
import frc.robot.Constants.AlgaeManipConstants.AlgaeManipAngleState;

public class AlgaeManipulator extends Subsystem<Double> {
  private final Motor leftMotor = Motor.neo(AlgaeManipConstants.LEFT_MOTOR_ID)
    .setCurrentLimit(AlgaeManipConstants.END_MOTORS_CURRENT_LIMIT);
  private final Motor rightMotor = Motor.neo(AlgaeManipConstants.RIGHT_MOTOR_ID)
    .setCurrentLimit(AlgaeManipConstants.END_MOTORS_CURRENT_LIMIT);
  private final Motor angleMotor = Motor.neo(AlgaeManipConstants.ANGLE_MOTOR_ID)
    .setCurrentLimit(AlgaeManipConstants.PIVOT_CURRENT_LIMIT)
    .setPID(AlgaeManipConstants.ANGLE_MOTOR_PID);

  public AlgaeManipulator() {
    super(AlgaeManipState.class, AlgaeManipAngleState.class);
  }

  public void updateMotors() {
    leftMotor.set(getState(AlgaeManipState.class).leftMotorSpeed);
    rightMotor.set(getState(AlgaeManipState.class).rightMotorSpeed);
    angleMotor.setReference(
      degreesToRotations(
        getState(AlgaeManipAngleState.class).position
      ),
      Motor.Control.POSITION
    );
  }

  public boolean isAtTarget() {
    return angleMotor.isAtTarget(getState(AlgaeManipAngleState.class).position);
  }

  public void togglePivotState() {
    AlgaeManipAngleState currentState = getState(AlgaeManipAngleState.class);
    setState(
      currentState == AlgaeManipAngleState.OUT
        ? AlgaeManipAngleState.UP
        : AlgaeManipAngleState.OUT
    );
  }

  private double degreesToRotations(double degrees) {
    return (degrees / 360) * AlgaeManipConstants.GEAR_RATIO;
  }

  public void stop() {
    leftMotor.stop();
    rightMotor.stop();
    angleMotor.stop();
  }
}
