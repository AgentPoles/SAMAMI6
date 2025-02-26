package jason.eis.movements;

import jason.eis.LocalMap;
import jason.eis.Point;
import java.util.*;
import java.util.logging.Logger;

public class PlannedMovement extends Movement {
  private final Map<String, List<String>> agentPaths = new HashMap<>();
  private final Map<String, Boolean> movingToDispenser = new HashMap<>();
  private final Map<String, String> lastFailedMove = new HashMap<>();

  public PlannedMovement(Map<String, LocalMap> agentMaps) {
    super(agentMaps);
  }

  @Override
  protected boolean isValidMove(String agName, String direction) {
    LocalMap map = getAgentMap(agName);
    Point currentPos = getCurrentPosition(agName);
    Point newPos = calculateNewPosition(currentPos, direction);

    // Check if the move would hit an obstacle or is otherwise invalid
    return (
      !map.isObstacle(newPos) && !direction.equals(lastFailedMove.get(agName))
    );
  }

  @Override
  public void recordSuccess(String agName) {
    lastFailedMove.remove(agName);
    List<String> path = agentPaths.get(agName);
    if (path != null && !path.isEmpty()) {
      path.remove(0);
      if (path.isEmpty()) {
        movingToDispenser.put(agName, false);
        agentPaths.remove(agName);
      }
    }
  }

  @Override
  public void recordFailure(
    String agName,
    String direction,
    String failureType
  ) {
    lastFailedMove.put(agName, direction);
    if (FAILED_FORBIDDEN.equals(failureType)) {
      // If the move was forbidden, clear the current path
      agentPaths.remove(agName);
      movingToDispenser.put(agName, false);
    }
  }

  public boolean findNearestDispenser(String agName) {
    // This will be implemented later with pathfinding logic
    // For now, just reset any existing movement
    agentPaths.remove(agName);
    movingToDispenser.put(agName, false);
    return false;
  }

  public String getNextMove(String agName) {
    List<String> path = agentPaths.get(agName);
    if (path == null || path.isEmpty()) {
      movingToDispenser.put(agName, false);
      return null;
    }

    String nextMove = path.get(0);
    if (isValidMove(agName, nextMove)) {
      return nextMove;
    } else {
      // If the next move is invalid, clear the path and try finding a new one
      agentPaths.remove(agName);
      movingToDispenser.put(agName, false);
      return null;
    }
  }

  public void moveSucceeded(String agName) {
    recordSuccess(agName);
  }

  public void moveFailed(String agName) {
    List<String> path = agentPaths.get(agName);
    if (path != null && !path.isEmpty()) {
      recordFailure(agName, path.get(0), FAILED_FORBIDDEN);
    }
  }

  public boolean isMovingToDispenser(String agName) {
    return Boolean.TRUE.equals(movingToDispenser.get(agName));
  }

  protected List<String> getCurrentPath(String agName) {
    return agentPaths.getOrDefault(agName, Collections.emptyList());
  }

  protected void setPath(String agName, List<String> path) {
    if (path != null && !path.isEmpty()) {
      agentPaths.put(agName, new ArrayList<>(path));
      movingToDispenser.put(agName, true);
    } else {
      agentPaths.remove(agName);
      movingToDispenser.put(agName, false);
    }
  }
}
