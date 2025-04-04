package frc.robot.subsystems;

import frc.lib.motor.Motor;
import frc.lib.subsystem.Subsystem;
import frc.robot.Constants.MotorTestConstants.MotorTestState;

// the purpose of this class is to test the subsystem framework
// we'll be controlling the falcon 500s that'll be used on the elevator

public class MotorTest extends Subsystem<Double> {
    private final Motor testMotor = Motor.falcon(9)
        .setCurrentLimit(40);
    private final Motor testMotor2 = Motor.falcon(10)
        .setCurrentLimit(40);

    public MotorTest() { super(MotorTestState.class); }

    public void updateMotors() {
        testMotor.set(getState(MotorTestState.class).speed);
        testMotor2.set(getState(MotorTestState.class).speed * -1);
    }
}
