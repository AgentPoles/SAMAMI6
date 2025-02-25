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
  protected LocalMap localMap;

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

  public Movement(LocalMap localMap) {
    this.localMap = localMap;
  }

  protected Point getCurrentPosition() {
    if (localMap != null) {
      return localMap.getCurrentPosition();
    }
    logger.warning(
      "Using default getCurrentPosition without LocalMap. Position might be inaccurate."
    );
    return new Point(0, 0);
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
}
