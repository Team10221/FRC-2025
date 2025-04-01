package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ElevatorConstants.ElevatorPosition;
import frc.robot.subsystems.ElevatorSubsystem;

public class MoveElevatorCommand extends Command {
    protected ElevatorSubsystem elevator;
    protected double pos;

    public MoveElevatorCommand(ElevatorSubsystem elevator, double targetPos) {
        this.elevator = elevator;
        this.pos = targetPos;
    }

    public MoveElevatorCommand(ElevatorSubsystem elevator, ElevatorPosition targetPos) {
        this(elevator, targetPos.getPosition());
    }

    @Override
    public void initialize() {
        elevator.setTargetPosition(pos);
    }

    @Override
    public boolean isFinished() {
        return elevator.isAtTargetPosition();
    }

    @Override
    public void end(boolean interrupted) {
        // do nothing, i think
    }
}
