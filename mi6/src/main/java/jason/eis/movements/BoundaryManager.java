package jason.eis.movements;

import jason.eis.LocalMap;
import jason.eis.Point;
import java.util.*;
import java.util.logging.Logger;
import java.util.stream.Collectors;

public class BoundaryManager {
  private static final Logger logger = Logger.getLogger(
    BoundaryManager.class.getName()
  );
  private static final boolean DEBUG = true;

  /**
   * Filters directions based on boundary considerations with fallback options
   * @param availableDirections Initial set of available directions
   * @param map Current map state
   * @param currentPos Agent's current position
   * @param previousPos Agent's previous position (can be null)
   * @param previousMove Previous move direction (can be null)
   * @return Filtered list of valid directions, never empty if input wasn't empty
   */
  public List<String> filterDirections(
    List<String> availableDirections,
    LocalMap map,
    Point currentPos,
    Point previousPos,
    String previousMove
  ) {
    if (availableDirections.isEmpty()) {
      return availableDirections;
    }

    // First try normal filtering
    List<String> filteredDirections = availableDirections
      .stream()
      .filter(
        dir -> {
          Point next = calculateNextPosition(currentPos, dir);
          boolean forbidden = map.isForbidden(next);

          if (DEBUG && forbidden) {
            logger.info(
              String.format(
                "FORBIDDEN MOVE - Direction %s to %s is forbidden",
                dir,
                next
              )
            );
          }

          return !forbidden;
        }
      )
      .filter(dir -> !wouldHitBoundary(map, currentPos, dir))
      .collect(Collectors.toList());

    // If we have valid directions, return them
    if (!filteredDirections.isEmpty()) {
      return filteredDirections;
    }

    // If we're stuck (same position) and have a previous move
    if (
      previousPos != null &&
      currentPos.equals(previousPos) &&
      previousMove != null
    ) {
      logger.info(
        "All directions filtered out and agent is stuck. Using optimistic filtering."
      );

      // Remove the previous move that got us stuck
      List<String> optimisticDirections = new ArrayList<>(availableDirections);
      optimisticDirections.remove(previousMove);

      if (!optimisticDirections.isEmpty()) {
        logger.info(
          "Allowing potentially risky directions: " + optimisticDirections
        );
        return optimisticDirections;
      }
    }

    // If all else fails, return all directions except previous move
    logger.info(
      "Falling back to all available directions except previous move"
    );
    List<String> fallbackDirections = new ArrayList<>(availableDirections);
    if (previousMove != null) {
      fallbackDirections.remove(previousMove);
    }
    return fallbackDirections.isEmpty()
      ? availableDirections
      : fallbackDirections;
  }

  /**
   * Checks if moving in a direction would hit a known boundary
   */
  private boolean wouldHitBoundary(
    LocalMap map,
    Point currentPos,
    String direction
  ) {
    Point nextPos = calculateNextPosition(currentPos, direction);
    Map<String, Point> boundaries = map.getConfirmedBoundariesPositions();

    if (boundaries.isEmpty()) {
      return false;
    }

    // Check if next position would hit any known boundary
    for (Map.Entry<String, Point> entry : boundaries.entrySet()) {
      Point boundaryPos = entry.getValue();
      String boundaryDir = entry.getKey();

      switch (boundaryDir) {
        case "n":
          if (nextPos.y <= boundaryPos.y) return true;
          break;
        case "s":
          if (nextPos.y >= boundaryPos.y) return true;
          break;
        case "e":
          if (nextPos.x >= boundaryPos.x) return true;
          break;
        case "w":
          if (nextPos.x <= boundaryPos.x) return true;
          break;
      }
    }

    return false;
  }

  private Point calculateNextPosition(Point current, String direction) {
    switch (direction) {
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
}
