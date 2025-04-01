package frc.robot.subsystems;

import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.motor.Motor;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.ElevatorConstants.ElevatorPosition;

public class ElevatorSubsystem extends SubsystemBase {
    Motor mainMotor, invertedMotor;
    double targetPos;

    public ElevatorSubsystem() {
        this.mainMotor = Motor.falcon(ElevatorConstants.MAIN_MOTOR_ID);
        this.invertedMotor = Motor.falcon(ElevatorConstants.OTHER_MOTOR_ID_THAT_WILL_BE_INVERTED_SOMEHOW);
    }

    /* in meters */
    public double getRealPosition() {
        return (
            (mainMotor.getPosition() / ElevatorConstants.ENCODER_TICKS_PER_METER)
            + -(invertedMotor.getPosition() / ElevatorConstants.ENCODER_TICKS_PER_METER)
        ) / 2;
    }

    /* in meters */
    public double getTargetPosition() {
        return targetPos;
    }

    public boolean isAtTargetPosition() {
        return mainMotor.isAtTarget(targetPos) || invertedMotor.isAtTarget(targetPos);
    }

    public void setTargetPosition(double target) {
        targetPos = target;
    }

    public void setTargetPosition(ElevatorPosition target) {
        this.setTargetPosition(target.getPosition());
    }

    public void setMotorVolts(double volts) {
        mainMotor.setVolts(volts);
        invertedMotor.setVolts(-volts);
    }

    @Override
    public void periodic() {
        // for zhe main motor
        double pided = ElevatorConstants.ELEVATOR_MOTOR_PID.calculate(getTargetPosition(), targetPos);
        double afterFeedForward = ElevatorConstants.ELEVATOR_FEEDFORWARD.calculate(pided);
        mainMotor.setVolts(afterFeedForward);
        invertedMotor.setVolts(-afterFeedForward);
    }

    public void stop() {
        mainMotor.stop();
        invertedMotor.stop();
    }
}
