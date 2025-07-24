package frc.robot.other;

import frc.robot.Constants.RobotState;
import java.util.*;

public class RobotStateMachine {
    public static class Transition {
        public final RobotState target;
        public final double time;

        public Transition(RobotState target, double time) {
            this.target = target;
            this.time = time;
        }
    }

    private static class PathNode {
        public final RobotState state;
        public final PathNode previous;

        public PathNode(RobotState state, PathNode previous) {
            this.state = state;
            this.previous = previous;
        }
    }

    private final Map<RobotState, Set<Transition>> stateGraph = new HashMap<>();

    public void addTransitions(RobotState state, Set<Transition> transitions) {
        stateGraph.put(state, transitions);
    }

    public List<RobotState> findPath(RobotState from, RobotState to) {
        if (from == to) {
            return Collections.singletonList(from);
        }

        Queue<PathNode> queue = new LinkedList<>();
        Set<RobotState> visited = new HashSet<>();

        queue.offer(new PathNode(from, null));
        visited.add(from);

        while (!queue.isEmpty()) {
            PathNode current = queue.poll();

            Set<Transition> transitions = stateGraph.get(current.state);
            if (transitions == null) continue;

            for (Transition transition : transitions) {
                if (visited.contains(transition.target)) continue;

                PathNode nextNode = new PathNode(transition.target, current);

                if (transition.target == to) {
                    List<RobotState> path = new ArrayList<>();
                    PathNode node = nextNode;
                    while (node != null) {
                        path.add(0, node.state);
                        node = node.previous;
                    }
                    return path;
                }

                visited.add(transition.target);
                queue.offer(nextNode);
            }
        }

        return null;
    }

    public Double getTransitionTime(RobotState from, RobotState to) {
        Set<Transition> transitions = stateGraph.get(from);
        if (transitions == null) return null;

        for (Transition transition : transitions) {
            if (transition.target == to) {
                return transition.time;
            }
        }
        return null;
    }
}
