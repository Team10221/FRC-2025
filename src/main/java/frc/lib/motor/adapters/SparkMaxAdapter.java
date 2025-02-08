package frc.lib.motor.adapters;

import com.revrobotics.spark.SparkMax;

import frc.lib.motor.MotorAdapter;
import frc.lib.motor.Motor.Control;
import frc.lib.util.PID;

import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.SoftLimitConfig;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkBase.ResetMode;


public class SparkMaxAdapter implements MotorAdapter {
  private SparkMax motor;
  private SparkMaxConfig config;

  public SparkMaxAdapter(SparkMax motor) {
    this.motor = motor;
    config = new SparkMaxConfig();
  }

  public void setPID(PID pid) {
    configure(config.apply(
      new ClosedLoopConfig().pid(
        pid.getP().orElse(0.0),
        pid.getI().orElse(0.0),
        pid.getD().orElse(0.0))
    ));
  }

  public void setReference(double reference, Control controlType) {
    motor.getClosedLoopController().setReference(reference,
      switch (controlType) {
        case POSITION -> ControlType.kPosition;
        case VELOCITY -> ControlType.kVelocity;
        case VOLTAGE -> ControlType.kVoltage;
      }
    );
  }

  public double getPosition() {
    return motor.getEncoder().getPosition();
  }

  public double getVelocity() {
    return motor.getEncoder().getVelocity();
  }

  public double getVoltage() {
    return motor.getBusVoltage() * motor.getAppliedOutput();
  }

  public void setInverted(boolean toInvert) {
    configure(config.inverted(toInvert));
  }

  public boolean isInverted() {
    return motor.configAccessor.getInverted();
  }

  public void setCurrentLimit(double limit) {
    configure(config.smartCurrentLimit((int) limit));
  }

  public void setForwardLimit(double forward) {
    configure(config.apply(
      new SoftLimitConfig()
        .forwardSoftLimitEnabled(true)
        .forwardSoftLimit(forward)
    ));
  }

  public void setBackLimit(double back) {
    configure(config.apply(
      new SoftLimitConfig()
        .reverseSoftLimitEnabled(true)
        .reverseSoftLimit(back)
    ));
  }

  public void setSoftLimits(double forward, double back) {
    setForwardLimit(forward);
    setBackLimit(back);
  }

  public void resetEncoder() {
    motor.getEncoder().setPosition(0);
  }

  private void configure(SparkBaseConfig config) {
    motor.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
  }
}