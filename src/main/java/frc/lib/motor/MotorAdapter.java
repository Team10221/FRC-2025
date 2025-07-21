package frc.lib.motor;

import frc.lib.motor.Motor.Control;
import frc.lib.util.PID;

public interface MotorAdapter<T> {
    void stop();
    void set(double speed);
    void resetEncoder();
    void setPID(PID pid);
    void setInverted(boolean toInvert);
    void setCurrentLimit(double limit);
    void setReference(double reference, Control controlType);
    void setSoftLimits(double forward, double back);
    void setForwardLimit(double forward);
    void setBackLimit(double back);
    double getPosition();
    double getVelocity();
    double getVoltage();
    boolean isInverted();
    T getMotorController();
    default void useExternalEncoder() {};
    default void setBrakeMode() {};
    default void setCoastMode() {};
}