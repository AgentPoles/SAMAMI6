package jason.eis.movements;

import jason.eis.LocalMap;
import jason.eis.MI6Model;
import jason.eis.Point;
import java.util.*;

public class PlannedMovement implements MovementStrategy {
  private final Search pathFinder;
  private final Map<String, MovementState> agentStates;
  private static final int PATH_TIMEOUT = 5000; // 5 seconds before recalculating path

  public PlannedMovement() {
    this.pathFinder = new Search();
    this.agentStates = new HashMap<>();
  }

  private static class MovementState {
    Point targetPosition;
    Search.TargetType targetType;
    List<String> plannedPath;
    long pathCalculatedTime;
    int currentPathIndex;

    MovementState(Point target, Search.TargetType type) {
      this.targetPosition = target;
      this.targetType = type;
      this.plannedPath = new ArrayList<>();
      this.pathCalculatedTime = 0;
      this.currentPathIndex = 0;
    }

    boolean needsNewPath() {
      return (
        plannedPath.isEmpty() ||
        currentPathIndex >= plannedPath.size() ||
        System.currentTimeMillis() - pathCalculatedTime > PATH_TIMEOUT
      );
    }

    String getNextMove() {
      return currentPathIndex < plannedPath.size()
        ? plannedPath.get(currentPathIndex)
        : null;
    }

    void incrementPathIndex() {
      currentPathIndex++;
    }
  }

  @Override
  public String getNextMove(String agName, LocalMap map) {
    MovementState state = agentStates.computeIfAbsent(
      agName,
      k -> new MovementState(null, null)
    );

    // If no target or path needs recalculation
    if (state.targetPosition == null || state.needsNewPath()) {
      Search.PathResult path = pathFinder.findPath(
        map.getCurrentPosition(),
        state.targetPosition,
        map,
        state.targetType
      );

      if (path.success) {
        state.plannedPath = path.directions;
        state.pathCalculatedTime = System.currentTimeMillis();
        state.currentPathIndex = 0;
      } else {
        return null; // No path found
      }
    }

    String nextMove = state.getNextMove();
    if (nextMove != null) {
      state.incrementPathIndex();
    }

    return nextMove;
  }

  public String getNextMove(String agName) {
    LocalMap map = MI6Model.getInstance().getAgentMap(agName);
    return getNextMove(agName, map);
  }

  public void setTarget(String agName, Point target, Search.TargetType type) {
    MovementState state = new MovementState(target, type);
    agentStates.put(agName, state);
  }

  public void clearTarget(String agName) {
    agentStates.remove(agName);
  }

  public boolean hasTarget(String agName) {
    return (
      agentStates.containsKey(agName) &&
      agentStates.get(agName).targetPosition != null
    );
  }

  public void moveSucceeded(String agName) {
    MovementState state = agentStates.get(agName);
    if (state != null) {
      state.incrementPathIndex();
    }
  }

  public void moveFailed(String agName) {
    // Clear current path on failure
    agentStates.remove(agName);
  }
}
