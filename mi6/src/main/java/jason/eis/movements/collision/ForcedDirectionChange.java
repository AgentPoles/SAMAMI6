package jason.eis.movements.collision;

import jason.eis.LocalMap;
import jason.eis.Point;
import java.util.*;
import java.util.logging.Logger;

public class ForcedDirectionChange {
  private static final Logger logger = Logger.getLogger(
    ForcedDirectionChange.class.getName()
  );
  private static final boolean DEBUG = false;
  private static final int DIRECTION_CHANGE_THRESHOLD = 7;
  private static final int MAX_TRIES = 4; // Maximum number of different directions to try

  public CollisionResolution checkCollision(
    String agentId,
    Point currentPos,
    String intendedDirection,
    LocalMap map,
    int size,
    String blockAttachment,
    List<String> availableDirections
  ) {
    try {
      String lastDirection = map.getLastDirection();
      if (lastDirection == null) {
        return null;
      }

      // Check if we should start forced direction change
      if (
        map.getSameDirectionCount() >= DIRECTION_CHANGE_THRESHOLD &&
        !map.isWatchingForcedChange()
      ) {
        debug("Starting forced direction change for agent %s", agentId);
        String oppositeDir = getOppositeDirection(lastDirection);
        map.startWatchingForcedChange();
        map.incrementForcedDirectionTry(oppositeDir);
        return new CollisionResolution(oppositeDir, "FORCED_CHANGE");
      }

      // If we're watching, continue trying until success or exhaustion
      if (map.isWatchingForcedChange()) {
        // If we've tried too many times, give up
        if (map.getForcedDirectionTryCount() >= MAX_TRIES) {
          debug(
            "Giving up on forced direction change after %d attempts",
            MAX_TRIES
          );
          map.stopWatchingForcedChange();
          return null;
        }

        // Try a new direction we haven't tried yet
        List<String> untried = new ArrayList<>(availableDirections);
        untried.removeAll(map.getTriedForcedDirections());
        untried.remove(lastDirection); // Don't try the direction we're trying to avoid

        if (!untried.isEmpty()) {
          String newDirection = untried.get(
            new Random().nextInt(untried.size())
          );
          debug("Trying new direction %s for forced change", newDirection);
          map.incrementForcedDirectionTry(newDirection);
          return new CollisionResolution(newDirection, "FORCED_CHANGE");
        } else {
          // No more directions to try
          debug("No more directions to try for forced change");
          map.stopWatchingForcedChange();
          return null;
        }
      }

      return null;
    } catch (Exception e) {
      logger.warning("Error in ForcedDirectionChange: " + e.getMessage());
      return null;
    }
  }

  private String getOppositeDirection(String direction) {
    if (direction == null) return null;
    switch (direction) {
      case "n":
        return "s";
      case "s":
        return "n";
      case "e":
        return "w";
      case "w":
        return "e";
      default:
        return null;
    }
  }

  private void debug(String format, Object... args) {
    if (DEBUG) {
      logger.fine(String.format(format, args));
    }
  }
}
