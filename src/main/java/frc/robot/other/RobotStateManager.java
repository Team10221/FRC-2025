package frc.robot.other;

import java.util.List;
import java.util.Set;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.RobotState;
import frc.robot.subsystems.AlgaeManipulator;
import frc.robot.subsystems.CoralManipulator;
import frc.robot.subsystems.Elevator;

public class RobotStateManager {
    private RobotStateMachine sm;
    private RobotState currentState;
    private CoralManipulator coralManipulator;
    private AlgaeManipulator algaeManipulator;
    private Elevator elevator;
    private Command currentCommand;
    private RobotState targetState;

    public RobotStateManager(
            Elevator elevator,
            AlgaeManipulator algaeManipulator,
            CoralManipulator coralManipulator) {
        this.elevator = elevator;
        this.algaeManipulator = algaeManipulator;
        this.coralManipulator = coralManipulator;
        this.currentState = RobotState.DEFAULT;
        this.sm = new RobotStateMachine();
        defineTransitions();
        applyState(currentState);
    }

    public void defineTransitions() {
        sm.addTransitions(RobotState.DEFAULT, Set.of(
                new RobotStateMachine.Transition(RobotState.CORAL_TRANSITION_IN, 0.2)));

        sm.addTransitions(RobotState.CORAL_TRANSITION_IN, Set.of(
                new RobotStateMachine.Transition(RobotState.DEFAULT, 0.2),
                new RobotStateMachine.Transition(RobotState.CORAL_TRANSITION_OUT, 0.5)));

        sm.addTransitions(RobotState.CORAL_TRANSITION_OUT, Set.of(
                new RobotStateMachine.Transition(RobotState.CORAL_TRANSITION_IN, 0.5),
                new RobotStateMachine.Transition(RobotState.CORAL_L2, 0.2),
                new RobotStateMachine.Transition(RobotState.CORAL_L3, 0.2),
                new RobotStateMachine.Transition(RobotState.CORAL_L4, 0.2),
                new RobotStateMachine.Transition(RobotState.ALGAE_OUTTAKE_PROCESSOR, 0.2),
                new RobotStateMachine.Transition(RobotState.ALGAE_REEF_INTAKE_L2, 0.2),
                new RobotStateMachine.Transition(RobotState.ALGAE_REEF_INTAKE_L3, 0.2)));

        sm.addTransitions(RobotState.CORAL_L2, Set.of(
                new RobotStateMachine.Transition(RobotState.CORAL_L3, 0.2),
                new RobotStateMachine.Transition(RobotState.CORAL_L4, 0.2),
                new RobotStateMachine.Transition(RobotState.CORAL_TRANSITION_OUT, 0.2),
                new RobotStateMachine.Transition(RobotState.ALGAE_OUTTAKE_PROCESSOR, 0.2),
                new RobotStateMachine.Transition(RobotState.ALGAE_REEF_INTAKE_L2, 0.2),
                new RobotStateMachine.Transition(RobotState.ALGAE_REEF_INTAKE_L3, 0.2)));

        sm.addTransitions(RobotState.CORAL_L3, Set.of(
                new RobotStateMachine.Transition(RobotState.CORAL_L4, 0.2),
                new RobotStateMachine.Transition(RobotState.CORAL_L2, 0.2),
                new RobotStateMachine.Transition(RobotState.CORAL_TRANSITION_OUT, 0.2),
                new RobotStateMachine.Transition(RobotState.ALGAE_OUTTAKE_PROCESSOR, 0.2),
                new RobotStateMachine.Transition(RobotState.ALGAE_REEF_INTAKE_L2, 0.2),
                new RobotStateMachine.Transition(RobotState.ALGAE_REEF_INTAKE_L3, 0.2)));

        sm.addTransitions(RobotState.CORAL_L4, Set.of(
                new RobotStateMachine.Transition(RobotState.CORAL_TRANSITION_OUT, 0.2),
                new RobotStateMachine.Transition(RobotState.CORAL_L2, 0.2),
                new RobotStateMachine.Transition(RobotState.CORAL_L3, 0.2),
                new RobotStateMachine.Transition(RobotState.ALGAE_OUTTAKE_PROCESSOR, 0.2),
                new RobotStateMachine.Transition(RobotState.ALGAE_REEF_INTAKE_L2, 0.2),
                new RobotStateMachine.Transition(RobotState.ALGAE_REEF_INTAKE_L3, 0.2)));

        sm.addTransitions(RobotState.ALGAE_OUTTAKE_PROCESSOR, Set.of(
                new RobotStateMachine.Transition(RobotState.ALGAE_REEF_INTAKE_L2, 0.2),
                new RobotStateMachine.Transition(RobotState.ALGAE_REEF_INTAKE_L3, 0.2),
                new RobotStateMachine.Transition(RobotState.CORAL_L2, 0.2),
                new RobotStateMachine.Transition(RobotState.CORAL_L3, 0.2),
                new RobotStateMachine.Transition(RobotState.CORAL_L4, 0.2),
                new RobotStateMachine.Transition(RobotState.CORAL_TRANSITION_OUT, 0.2)));

        sm.addTransitions(RobotState.ALGAE_REEF_INTAKE_L2, Set.of(
                new RobotStateMachine.Transition(RobotState.ALGAE_REEF_INTAKE_L3, 0.2),
                new RobotStateMachine.Transition(RobotState.ALGAE_OUTTAKE_PROCESSOR, 0.2),
                new RobotStateMachine.Transition(RobotState.CORAL_L2, 0.2),
                new RobotStateMachine.Transition(RobotState.CORAL_L3, 0.2),
                new RobotStateMachine.Transition(RobotState.CORAL_L4, 0.2),
                new RobotStateMachine.Transition(RobotState.CORAL_TRANSITION_OUT, 0.2)));

        sm.addTransitions(RobotState.ALGAE_REEF_INTAKE_L3, Set.of(
                new RobotStateMachine.Transition(RobotState.ALGAE_REEF_INTAKE_L2, 0.2),
                new RobotStateMachine.Transition(RobotState.ALGAE_OUTTAKE_PROCESSOR, 0.2),
                new RobotStateMachine.Transition(RobotState.CORAL_L2, 0.2),
                new RobotStateMachine.Transition(RobotState.CORAL_L3, 0.2),
                new RobotStateMachine.Transition(RobotState.CORAL_L4, 0.2),
                new RobotStateMachine.Transition(RobotState.CORAL_TRANSITION_OUT, 0.2)));
    }

    public void changeState(RobotState target) {
        if (target == currentState) {
            return;
        }

        if (targetState == target) {
            return;
        }

        if (currentCommand != null && !currentCommand.isFinished()) {
            currentCommand.cancel();
        }

        List<RobotState> path = sm.findPath(currentState, target);
        if (path == null || path.size() < 2) {
            return;
        }

        targetState = target;
        currentCommand = createPathCommand(path);
        CommandScheduler.getInstance().schedule(currentCommand);
    }

    private Command createPathCommand(List<RobotState> path) {
        SequentialCommandGroup command = new SequentialCommandGroup();

        for (int i = 1; i < path.size(); i++) {
            RobotState nextState = path.get(i);
            Double time = sm.getTransitionTime(path.get(i - 1), nextState);

            command.addCommands(
                    new InstantCommand(() -> {
                        applyState(nextState);
                        currentState = nextState;
                    }),
                    new WaitCommand(time != null ? time : 0.2));
        }

        command.addCommands(new InstantCommand(() -> targetState = null));
        return command;
    }

    private void applyState(RobotState state) {
        elevator.setState(state.elevatorState);
        algaeManipulator.setState(state.algaeManipAngleState);
        coralManipulator.setState(state.coralManipAngleState);
    }

    public RobotState getCurrentState() {
        return currentState;
    }

    public boolean isTransitioning() {
        return currentCommand != null && !currentCommand.isFinished();
    }
}
