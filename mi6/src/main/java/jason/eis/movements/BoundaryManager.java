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
    try {
      // Validate inputs
      if (availableDirections == null) {
        logger.severe("Available directions list is null");
        return new ArrayList<>();
      }
      if (map == null) {
        logger.severe("Map is null");
        return availableDirections;
      }
      if (currentPos == null) {
        logger.severe("Current position is null");
        return availableDirections;
      }

      if (DEBUG) {
        logger.info(
          String.format(
            "Filtering directions - Current: %s, Previous: %s, PrevMove: %s, Available: %s",
            currentPos,
            previousPos,
            previousMove,
            availableDirections
          )
        );
      }

      // First try normal filtering
      List<String> filteredDirections = availableDirections
        .stream()
        .filter(
          dir -> {
            try {
              Point next = calculateNextPosition(currentPos, dir);
              boolean forbidden = map.isForbidden(next);
              boolean hitsBoundary = wouldHitBoundary(map, currentPos, dir);

              if (DEBUG) {
                logger.info(
                  String.format(
                    "Direction %s to %s - Forbidden: %b, Hits Boundary: %b",
                    dir,
                    next,
                    forbidden,
                    hitsBoundary
                  )
                );
              }

              return !forbidden && !hitsBoundary;
            } catch (Exception e) {
              logger.warning(
                String.format(
                  "Error checking direction %s: %s",
                  dir,
                  e.getMessage()
                )
              );
              return false;
            }
          }
        )
        .collect(Collectors.toList());

      if (!filteredDirections.isEmpty()) {
        if (DEBUG) {
          logger.info("Filtered directions: " + filteredDirections);
        }
        return filteredDirections;
      }

      // Handle stuck case
      if (
        previousPos != null &&
        currentPos.equals(previousPos) &&
        previousMove != null
      ) {
        if (DEBUG) {
          logger.info(
            String.format(
              "Agent stuck at %s, removing previous move %s",
              currentPos,
              previousMove
            )
          );
        }

        List<String> optimisticDirections = new ArrayList<>(
          availableDirections
        );
        optimisticDirections.remove(previousMove);

        if (!optimisticDirections.isEmpty()) {
          logger.info("Using optimistic directions: " + optimisticDirections);
          return optimisticDirections;
        }
      }

      // Fallback case
      List<String> fallbackDirections = new ArrayList<>(availableDirections);
      if (previousMove != null) {
        fallbackDirections.remove(previousMove);
      }

      if (DEBUG) {
        logger.info(
          "Using fallback directions: " +
          (
            fallbackDirections.isEmpty()
              ? availableDirections
              : fallbackDirections
          )
        );
      }

      return fallbackDirections.isEmpty()
        ? availableDirections
        : fallbackDirections;
    } catch (Exception e) {
      logger.severe("Critical error in filterDirections: " + e.getMessage());
      e.printStackTrace();
      return availableDirections; // Return original list in case of critical error
    }
  }

  /**
   * Checks if moving in a direction would hit a known boundary
   */
  private boolean wouldHitBoundary(
    LocalMap map,
    Point currentPos,
    String direction
  ) {
    try {
      Point nextPos = calculateNextPosition(currentPos, direction);
      Map<String, Point> boundaries = map.getConfirmedBoundariesPositions();

      if (boundaries.isEmpty()) {
        return false;
      }

      for (Map.Entry<String, Point> entry : boundaries.entrySet()) {
        Point boundaryPos = entry.getValue();
        String boundaryDir = entry.getKey();

        boolean hits = false;
        switch (boundaryDir) {
          case "n":
            hits = nextPos.y <= boundaryPos.y;
            break;
          case "s":
            hits = nextPos.y >= boundaryPos.y;
            break;
          case "e":
            hits = nextPos.x >= boundaryPos.x;
            break;
          case "w":
            hits = nextPos.x <= boundaryPos.x;
            break;
        }

        if (hits && DEBUG) {
          logger.info(
            String.format(
              "Hit boundary at %s moving %s (boundary: %s %s)",
              nextPos,
              direction,
              boundaryDir,
              boundaryPos
            )
          );
        }

        if (hits) return true;
      }

      return false;
    } catch (Exception e) {
      logger.warning("Error checking boundary collision: " + e.getMessage());
      return true; // Safer to assume boundary hit on error
    }
  }

  private Point calculateNextPosition(Point current, String direction) {
    if (current == null || direction == null) {
      throw new IllegalArgumentException(
        "Current position or direction is null"
      );
    }

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
        throw new IllegalArgumentException("Invalid direction: " + direction);
    }
  }
}
