package frc.robot.other;

import java.util.List;
import java.util.Set;

import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants.RobotState;
import frc.robot.other.RobotStateMachine.Transition;
import frc.robot.subsystems.AlgaeManipulator;
import frc.robot.subsystems.CoralManipulator;
import frc.robot.subsystems.Elevator;

public class RobotStateManager {
    private RobotStateMachine sm;
    private RobotState currentState;
    private CoralManipulator coralManipulator;
    private AlgaeManipulator algaeManipulator;
    private Elevator elevator;
    
    private List<RobotState> currentPath;
    private int pathIndex;
    private double stateChangeTime;
    private double transitionDelay;

    public RobotStateManager(
        Elevator elevator,
        AlgaeManipulator algaeManipulator,
        CoralManipulator coralManipulator
    ) {
        this.elevator = elevator;
        this.algaeManipulator = algaeManipulator;
        this.coralManipulator = coralManipulator;
        this.currentState = RobotState.DEFAULT;
        this.stateChangeTime = 0;
        this.transitionDelay = 0;

        sm = new RobotStateMachine();
        defineTransitions();
        applyState(currentState);
    }

    public void defineTransitions() {
        sm.addTransitions(RobotState.DEFAULT, Set.of(
            new Transition(RobotState.CORAL_TRANSITION_IN, 0.2)
        ));

        sm.addTransitions(RobotState.CORAL_TRANSITION_IN, Set.of(
            new Transition(RobotState.DEFAULT, 0.2),
            new Transition(RobotState.CORAL_TRANSITION_OUT, 0.5)
        ));

        sm.addTransitions(RobotState.CORAL_TRANSITION_OUT, Set.of(
            new Transition(RobotState.CORAL_TRANSITION_IN, 0.5),
            new Transition(RobotState.CORAL_L2, 0.2),
            new Transition(RobotState.CORAL_L3, 0.2),
            new Transition(RobotState.CORAL_L4, 0.2),
            new Transition(RobotState.ALGAE_OUTTAKE_PROCESSOR, 0.2),
            new Transition(RobotState.ALGAE_REEF_INTAKE_L2, 0.2),
            new Transition(RobotState.ALGAE_REEF_INTAKE_L3, 0.2)
        ));

        sm.addTransitions(RobotState.CORAL_L2, Set.of(
            new Transition(RobotState.CORAL_L3, 0.2),
            new Transition(RobotState.CORAL_L4, 0.2),
            new Transition(RobotState.CORAL_TRANSITION_OUT, 0.2),
            new Transition(RobotState.ALGAE_OUTTAKE_PROCESSOR, 0.2),
            new Transition(RobotState.ALGAE_REEF_INTAKE_L2, 0.2),
            new Transition(RobotState.ALGAE_REEF_INTAKE_L3, 0.2)
        ));

        sm.addTransitions(RobotState.CORAL_L3, Set.of(
            new Transition(RobotState.CORAL_L4, 0.2),
            new Transition(RobotState.CORAL_L2, 0.2),
            new Transition(RobotState.CORAL_TRANSITION_OUT, 0.2),
            new Transition(RobotState.ALGAE_OUTTAKE_PROCESSOR, 0.2),
            new Transition(RobotState.ALGAE_REEF_INTAKE_L2, 0.2),
            new Transition(RobotState.ALGAE_REEF_INTAKE_L3, 0.2)
        ));

        sm.addTransitions(RobotState.CORAL_L4, Set.of(
            new Transition(RobotState.CORAL_TRANSITION_OUT, 0.2),
            new Transition(RobotState.CORAL_L2, 0.2),
            new Transition(RobotState.CORAL_L3, 0.2),
            new Transition(RobotState.ALGAE_OUTTAKE_PROCESSOR, 0.2),
            new Transition(RobotState.ALGAE_REEF_INTAKE_L2, 0.2),
            new Transition(RobotState.ALGAE_REEF_INTAKE_L3, 0.2)
        ));

        sm.addTransitions(RobotState.ALGAE_OUTTAKE_PROCESSOR, Set.of(
            new Transition(RobotState.ALGAE_REEF_INTAKE_L2, 0.2),
            new Transition(RobotState.ALGAE_REEF_INTAKE_L3, 0.2),
            new Transition(RobotState.CORAL_L2, 0.2),
            new Transition(RobotState.CORAL_L3, 0.2),
            new Transition(RobotState.CORAL_L4, 0.2),
            new Transition(RobotState.CORAL_TRANSITION_OUT, 0.2)
        ));

        sm.addTransitions(RobotState.ALGAE_REEF_INTAKE_L2, Set.of(
            new Transition(RobotState.ALGAE_REEF_INTAKE_L3, 0.2),
            new Transition(RobotState.ALGAE_OUTTAKE_PROCESSOR, 0.2),
            new Transition(RobotState.CORAL_L2, 0.2),
            new Transition(RobotState.CORAL_L3, 0.2),
            new Transition(RobotState.CORAL_L4, 0.2),
            new Transition(RobotState.CORAL_TRANSITION_OUT, 0.2)
        ));

        sm.addTransitions(RobotState.ALGAE_REEF_INTAKE_L3, Set.of(
            new Transition(RobotState.ALGAE_REEF_INTAKE_L2, 0.2),
            new Transition(RobotState.ALGAE_OUTTAKE_PROCESSOR, 0.2),
            new Transition(RobotState.CORAL_L2, 0.2),
            new Transition(RobotState.CORAL_L3, 0.2),
            new Transition(RobotState.CORAL_L4, 0.2),
            new Transition(RobotState.CORAL_TRANSITION_OUT, 0.2)
        ));
    }

    public void changeState(RobotState target) {
        if (Timer.getFPGATimestamp() < stateChangeTime + transitionDelay) {
            return;
        }
        
        if (currentPath != null && pathIndex < currentPath.size()) {
            return;
        }
        
        if (target == currentState) {
            return;
        }
        
        List<RobotState> path = sm.findPath(currentState, target);
        if (path == null || path.size() < 2) {
            return;
        }
        
        currentPath = path;
        pathIndex = 1;
        executeNextTransition();
    }

    public void periodic() {
        if (currentPath != null && pathIndex < currentPath.size() && 
            Timer.getFPGATimestamp() >= stateChangeTime + transitionDelay) {
            executeNextTransition();
        }
    }

    private void executeNextTransition() {
        RobotState nextState = currentPath.get(pathIndex);
        Double time = sm.getTransitionTime(currentState, nextState);
        
        if (time == null) {
            currentPath = null;
            return;
        }
        
        applyState(nextState);
        currentState = nextState;
        stateChangeTime = Timer.getFPGATimestamp();
        transitionDelay = time;
        
        pathIndex++;
        if (pathIndex >= currentPath.size()) {
            currentPath = null;
        }
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
        return currentPath != null && pathIndex < currentPath.size();
    }
}