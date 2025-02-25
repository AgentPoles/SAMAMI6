package jason.eis.movements;

import jason.eis.LocalMap;
import jason.eis.Point;
import java.util.*;
import java.util.logging.Logger;

public class PlannedMovement extends Movement {
  private final Map<String, List<String>> agentPaths = new HashMap<>();
  private final Map<String, Boolean> movingToDispenser = new HashMap<>();
  private final Map<String, LocalMap> agentMaps;

  public PlannedMovement(Map<String, LocalMap> agentMaps) {
    super(null);
    this.agentMaps = agentMaps;
  }

  @Override
  protected Point getCurrentPosition() {
    if (localMap != null) {
      return localMap.getCurrentPosition();
    }
    return new Point(0, 0);
  }

  public boolean findNearestDispenser(String agName) {
    if (Boolean.TRUE.equals(movingToDispenser.get(agName))) {
      logger.info("Agent " + agName + " is already moving to a dispenser.");
      return true;
    }

    LocalMap map = agentMaps.get(agName);
    if (map == null) return false;

    List<Point> dispensers = map.findNearestDispenser(null);
    if (dispensers.isEmpty()) {
      agentPaths.remove(agName);
      return false;
    }

    List<String> path = map.findPathTo(dispensers.get(0));
    if (path != null && !path.isEmpty()) {
      agentPaths.put(agName, path);
      movingToDispenser.put(agName, true);
      return true;
    }

    return false;
  }

  public String getNextMove(String agName) {
    List<String> path = agentPaths.get(agName);
    if (path == null || path.isEmpty()) {
      movingToDispenser.put(agName, false);
      return null;
    }
    return path.get(0);
  }

  public void moveSucceeded(String agName) {
    List<String> path = agentPaths.get(agName);
    if (path != null && !path.isEmpty()) {
      path.remove(0);
      if (path.isEmpty()) {
        movingToDispenser.put(agName, false);
      }
    }
  }

  public void moveFailed(String agName) {
    agentPaths.remove(agName);
    movingToDispenser.put(agName, false);
  }

  public boolean isMovingToDispenser(String agName) {
    return Boolean.TRUE.equals(movingToDispenser.get(agName));
  }
}
