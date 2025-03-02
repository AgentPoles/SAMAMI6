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
   * @param agentId Agent's ID
   * @param availableDirections Initial set of available directions
   * @param map Current map state
   * @param currentPos Agent's current position
   * @param previousPos Agent's previous position (can be null)
   * @param previousMove Previous move direction (can be null)
   * @return Filtered list of valid directions, never empty if input wasn't empty
   */
  public List<String> filterDirections(
    String agentId,
    List<String> availableDirections,
    LocalMap map,
    Point currentPos,
    Point previousPos,
    String previousMove
  ) {
    try {
      if (DEBUG) {
        logger.info(
          String.format(
            "[Agent %s] === Starting Direction Filtering ===",
            agentId
          )
        );
        logger.info(
          String.format(
            "[Agent %s] Input - Current: %s, Previous: %s, PrevMove: %s",
            agentId,
            currentPos,
            previousPos,
            previousMove
          )
        );
        logger.info(
          String.format(
            "[Agent %s] Available directions: %s",
            agentId,
            availableDirections
          )
        );
      }

      // Validate inputs
      if (availableDirections == null) {
        logger.severe(
          String.format("[Agent %s] Available directions list is null", agentId)
        );
        return new ArrayList<>();
      }
      if (map == null) {
        logger.severe(String.format("[Agent %s] Map is null", agentId));
        return availableDirections;
      }
      if (currentPos == null) {
        logger.severe(
          String.format("[Agent %s] Current position is null", agentId)
        );
        return availableDirections;
      }

      if (DEBUG) {
        logger.info(
          String.format(
            "[Agent %s] Filtering directions - Current: %s, Previous: %s, PrevMove: %s, Available: %s",
            agentId,
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
              boolean hitsBoundary = wouldHitBoundary(
                agentId,
                map,
                currentPos,
                dir
              );

              if (DEBUG) {
                logger.info(
                  String.format(
                    "[Agent %s] Checking direction %s to %s - Forbidden: %b, Hits Boundary: %b",
                    agentId,
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
                  "[Agent %s] Error checking direction %s: %s",
                  agentId,
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
          logger.info(
            String.format("[Agent %s] === Filtering Result ===", agentId)
          );
          logger.info(
            String.format(
              "[Agent %s] Filtered directions: %s",
              agentId,
              filteredDirections
            )
          );
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
              "[Agent %s] Agent stuck at %s, removing previous move %s",
              agentId,
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
          logger.info(
            String.format(
              "[Agent %s] Using optimistic directions: %s",
              agentId,
              optimisticDirections
            )
          );
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
          String.format(
            "[Agent %s] Using fallback directions: %s",
            agentId,
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
      logger.severe(
        String.format(
          "[Agent %s] Critical error in filterDirections: %s",
          agentId,
          e.getMessage()
        )
      );
      e.printStackTrace();
      return availableDirections; // Return original list in case of critical error
    }
  }

  /**
   * Checks if moving in a direction would hit a known boundary
   */
  private boolean wouldHitBoundary(
    String agentId,
    LocalMap map,
    Point currentPos,
    String direction
  ) {
    try {
      Point nextPos = calculateNextPosition(currentPos, direction);
      Map<String, Point> boundaries = map.getConfirmedBoundariesPositions();

      if (boundaries.isEmpty()) {
        if (DEBUG) logger.info(
          String.format("[Agent %s] No confirmed boundaries yet", agentId)
        );
        return false;
      }

      if (DEBUG) {
        logger.info(
          String.format(
            "[Agent %s] Checking boundary collision - Current: %s, Direction: %s, Next: %s",
            agentId,
            currentPos,
            direction,
            nextPos
          )
        );
        logger.info(
          String.format(
            "[Agent %s] Known boundaries: %s",
            agentId,
            boundaries
              .entrySet()
              .stream()
              .map(e -> e.getKey() + "=" + e.getValue())
              .collect(Collectors.joining(", "))
          )
        );
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

        if (DEBUG) {
          logger.info(
            String.format(
              "[Agent %s] Boundary check - Direction: %s, Boundary: %s at %s, Would Hit: %b",
              agentId,
              direction,
              boundaryDir,
              boundaryPos,
              hits
            )
          );
        }

        if (hits) {
          logger.warning(
            String.format(
              "[Agent %s] BOUNDARY HIT DETECTED - Pos: %s, Dir: %s, Next: %s, Boundary: %s at %s",
              agentId,
              currentPos,
              direction,
              nextPos,
              boundaryDir,
              boundaryPos
            )
          );
          return true;
        }
      }

      return false;
    } catch (Exception e) {
      logger.severe(
        String.format(
          "[Agent %s] Error in boundary check: %s",
          agentId,
          e.getMessage()
        )
      );
      e.printStackTrace();
      return true;
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
