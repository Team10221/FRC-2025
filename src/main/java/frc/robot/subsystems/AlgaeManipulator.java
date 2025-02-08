package frc.robot.subsystems;

import frc.lib.motor.Motor;
import frc.lib.subsystem.Subsystem;
import frc.robot.Constants.AlgaeManipulatorConstants;
import frc.robot.Constants.AlgaeManipulatorConstants.ManipulatorState;
import frc.robot.Constants.AlgaeManipulatorConstants.AngleState;

public class AlgaeManipulator extends Subsystem<Double> {
  private final Motor leftMotor = Motor.neo(AlgaeManipulatorConstants.LEFT_MOTOR_ID);
  private final Motor rightMotor = Motor.neo(AlgaeManipulatorConstants.RIGHT_MOTOR_ID);
  private final Motor angleMotor = Motor.neo(AlgaeManipulatorConstants.ANGLE_MOTOR_ID)
      .setPID(AlgaeManipulatorConstants.ANGLE_MOTOR_PID);

  public AlgaeManipulator() {
    super(ManipulatorState.class, AngleState.class);
  }

  public void updateMotors() {
    leftMotor.set(getState(ManipulatorState.class).leftMotorSpeed);
    rightMotor.set(getState(ManipulatorState.class).rightMotorSpeed);
    angleMotor.setRef(getState(AngleState.class).position);
  }

  public boolean isAtTarget() {
    return angleMotor.isAtTarget(getState(AngleState.class).position);
  }
}
