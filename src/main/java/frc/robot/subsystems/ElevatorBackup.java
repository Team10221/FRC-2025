package frc.robot.subsystems;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.ElevatorConstants.ElevatorState;

// in case our normal elevator code doesn't work
public class ElevatorBackup extends SubsystemBase {
    private final TalonFX leaderMotor;
    private final TalonFX followerMotor;
    private ElevatorState elevatorState;

    public ElevatorBackup() {
        leaderMotor = new TalonFX(9);
        followerMotor = new TalonFX(10);

        leaderMotor.getConfigurator().apply(new TalonFXConfiguration());
        followerMotor.getConfigurator().apply(new TalonFXConfiguration());

        elevatorState = ElevatorState.DOWN;

        var slot0Configs = new Slot0Configs();
        slot0Configs.kP = ElevatorConstants.ELEVATOR_PID.getP().orElse(0.0);
        slot0Configs.kI = ElevatorConstants.ELEVATOR_PID.getI().orElse(0.0);
        slot0Configs.kD = ElevatorConstants.ELEVATOR_PID.getD().orElse(0.0); 

        leaderMotor.getConfigurator().apply(slot0Configs);

        leaderMotor.setPosition(0);
        followerMotor.setPosition(0);

        followerMotor.setControl(
            new Follower(
                9, 
                true
            )
        );
    }

    public void setState(ElevatorState elevatorState) {
        this.elevatorState = elevatorState;
    }

    @Override
    public void periodic() {
        leaderMotor.setControl(new PositionVoltage(heightToRotations(elevatorState.height)));
    }

    public double heightToRotations(double heightMeters) {
        return (heightMeters / ElevatorConstants.SPROCKET_CIRCUMFERENCE) * ElevatorConstants.GEAR_RATIO;
    }

    public double rotationsToHeight(double rotations) {
        return (rotations / ElevatorConstants.GEAR_RATIO) * ElevatorConstants.SPROCKET_CIRCUMFERENCE;
    } 
}
