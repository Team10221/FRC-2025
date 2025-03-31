package frc.lib.motor.adapters;

import com.thethriftybot.ThriftyNova;
import com.thethriftybot.ThriftyNova.CurrentType;

import frc.lib.motor.Motor.Control;
import frc.lib.motor.MotorAdapter;
import frc.lib.util.PID;

public class ThriftyNovaAdapter implements MotorAdapter<ThriftyNova> {
    private ThriftyNova motor;

    public ThriftyNovaAdapter(ThriftyNova motor) {
        this.motor = motor;
    }

    public void setPID(PID pid) {
        motor.pid0
            .setP(pid.getP().orElse(0.0))
            .setI(pid.getI().orElse(0.0))
            .setD(pid.getD().orElse(0.0))
            .setFF(pid.getF().orElse(0.0));
    }

    public void setReference(double reference, Control controlType) {
        // hahaha dont need to implement
        // we arent needing to set reference this yr for novas
    }

    public double getPosition() {
        return motor.getPosition();
    }

    public double getVelocity() {
        return motor.getVelocity();
    }

    public double getVoltage() {
        return motor.getVoltage();
    }

    public void setInverted() {
        // might be useful
        motor.setInverted(true);

        // alternatively:
        // motor.setInverted(!motor.getInverted());
    }

    public void setInverted(boolean inverted) {
        motor.setInverted(inverted);
    }

    public boolean isInverted() {
        return motor.getInverted();
    }

    public void setCurrentLimit(double limit) {
        motor.setMaxCurrent(CurrentType.STATOR, limit);
    }

    public void setForwardLimit(double forward) {
       // for the thrifty nova you cant set specific forward/backward limits
       // so instead i'll just point this to setCurrentLimit
       setCurrentLimit(forward); 
    }

    public void setBackLimit(double back) {
        setCurrentLimit(back);
    }

    public void setSoftLimits(double forward, double back) {}

    public void resetEncoder() {
        motor.zeroAbsEncoder();
    }

    public ThriftyNova getMotorController() {
        return motor;
    }

    public void stop() {
        motor.stopMotor();
    }

    public void set(double speed) {
        motor.set(speed);
    }
}