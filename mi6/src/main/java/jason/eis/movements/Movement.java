package jason.eis.movements;

import jason.eis.LocalMap;
import jason.eis.Point;
import java.util.*;
import java.util.logging.Logger;

public abstract class Movement {
  protected static final Logger logger = Logger.getLogger(
    Movement.class.getName()
  );
  protected static final String[] DIRECTIONS = { "n", "s", "e", "w" };
  protected final Random random = new Random();
  protected final Map<String, LocalMap> agentMaps;

  // Pre-compute direction opposites for O(1) lookup
  protected static final Map<String, String> OPPOSITE_DIRECTIONS = new HashMap<>(
    4
  ) {

    {
      put("n", "s");
      put("s", "n");
      put("e", "w");
      put("w", "e");
    }
  };

  // Movement status constants
  protected static final String FAILED_PATH = "failed_path";
  protected static final String FAILED_FORBIDDEN = "failed_forbidden";
  protected static final String FAILED_PARAMETER = "failed_parameter";

  public Movement(Map<String, LocalMap> agentMaps) {
    this.agentMaps = agentMaps;
  }

  protected LocalMap getAgentMap(String agName) {
    LocalMap map = agentMaps.get(agName);
    if (map == null) {
      throw new IllegalStateException("No map found for agent: " + agName);
    }
    return map;
  }

  protected Point getCurrentPosition(String agName) {
    try {
      LocalMap map = getAgentMap(agName);
      return map.getCurrentPosition();
    } catch (Exception e) {
      logger.warning(
        "Error getting position for agent " + agName + ": " + e.getMessage()
      );
      return new Point(0, 0);
    }
  }

  protected List<String> getPerpendicularDirections(String direction) {
    List<String> perpDirs = new ArrayList<>(2);
    if ("n".equals(direction) || "s".equals(direction)) {
      perpDirs.add("e");
      perpDirs.add("w");
    } else {
      perpDirs.add("n");
      perpDirs.add("s");
    }
    return perpDirs;
  }

  protected boolean isTowardsTarget(
    String direction,
    int targetX,
    int targetY
  ) {
    return (
      (direction.equals("e") && targetX > 0) ||
      (direction.equals("w") && targetX < 0) ||
      (direction.equals("s") && targetY > 0) ||
      (direction.equals("n") && targetY < 0)
    );
  }

  protected Point calculateNewPosition(Point current, String direction) {
    switch (direction.toLowerCase()) {
      case "n":
        return new Point(current.x, current.y - 1);
      case "s":
        return new Point(current.x, current.y + 1);
      case "e":
        return new Point(current.x + 1, current.y);
      case "w":
        return new Point(current.x - 1, current.y);
      default:
        return current;
    }
  }

  protected String getRandomDirection() {
    return DIRECTIONS[random.nextInt(DIRECTIONS.length)];
  }

  protected String getOppositeDirection(String direction) {
    return OPPOSITE_DIRECTIONS.get(direction);
  }

  // Make these abstract methods that subclasses must implement
  public abstract void recordSuccess(String agName);

  public abstract void recordFailure(
    String agName,
    String direction,
    String failureType
  );

  protected abstract boolean isValidMove(String agName, String direction);
  // ... rest of the existing Movement class code ...
}
