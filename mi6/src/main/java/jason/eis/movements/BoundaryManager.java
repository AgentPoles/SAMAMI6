package jason.eis.movements;

import jason.eis.LocalMap;
import jason.eis.LocalMap.MovementRecord;
import jason.eis.LocalMap.ObstacleInfo;
import jason.eis.Point;
import java.util.*;
import java.util.logging.Logger;
import java.util.stream.Collectors;

public class BoundaryManager {
  private static final Logger logger = Logger.getLogger(
    BoundaryManager.class.getName()
  );
  private static final boolean DEBUG = true;
  private static final int BOUNDARY_SAFETY_MARGIN = 2;
  private static final double BOUNDARY_SCORE_THRESHOLD = 0.6;

  /**
   * Filters directions based on boundary considerations with fallback options
   * @param agentId Agent's ID
   * @param availableDirections Initial set of available directions
   * @param map Current map state
   * @return Filtered list of valid directions, never empty if input wasn't empty
   */
  public List<String> filterDirections(
    String agentId,
    List<String> availableDirections,
    LocalMap map
  ) {
    try {
      // Get positions and previous move from map
      Point currentPos = map.getCurrentPosition();
      Point previousPos = map.getLastPosition();
      String previousMove = map.getLastDirection();

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

      // Check for nearby agents and boundaries
      Map<Point, ObstacleInfo> dynamicObstacles = map.getDynamicObstacles();
      Map<String, Point> boundaries = map.getConfirmedBoundariesPositions();

      // Score-based filtering when near boundaries and agents
      if (!boundaries.isEmpty() && !dynamicObstacles.isEmpty()) {
        return filterWithScoring(agentId, availableDirections, map);
      }

      // Regular filtering if no complex situation
      List<String> filteredDirections = availableDirections
        .stream()
        .filter(
          dir -> isDirectionSafe(agentId, map, currentPos, dir, boundaries)
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

      // Enhanced stuck handling
      return handleStuckNearBoundary(
        agentId,
        availableDirections,
        map,
        currentPos,
        previousPos,
        previousMove,
        boundaries
      );
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

  private List<String> filterWithScoring(
    String agentId,
    List<String> availableDirections,
    LocalMap map
  ) {
    Point currentPos = map.getCurrentPosition();
    Map<Point, ObstacleInfo> dynamicObstacles = map.getDynamicObstacles();
    Map<String, Point> boundaries = map.getConfirmedBoundariesPositions();

    Map<String, Double> directionScores = new HashMap<>();

    for (String direction : availableDirections) {
      Point nextPos = calculateNextPosition(currentPos, direction);
      double score = calculateDirectionScore(map, nextPos, direction);
      directionScores.put(direction, score);
    }

    // Filter directions with acceptable scores
    List<String> validDirections = directionScores
      .entrySet()
      .stream()
      .filter(e -> e.getValue() >= BOUNDARY_SCORE_THRESHOLD)
      .sorted(Map.Entry.<String, Double>comparingByValue().reversed())
      .map(Map.Entry::getKey)
      .collect(Collectors.toList());

    if (!validDirections.isEmpty()) {
      return validDirections;
    }

    // Fallback to best available direction if none meet threshold
    return directionScores
      .entrySet()
      .stream()
      .max(Map.Entry.comparingByValue())
      .map(e -> Collections.singletonList(e.getKey()))
      .orElse(new ArrayList<>(availableDirections));
  }

  private double calculateDirectionScore(
    LocalMap map,
    Point nextPos,
    String direction
  ) {
    Point currentPos = map.getCurrentPosition();
    Map<Point, ObstacleInfo> dynamicObstacles = map.getDynamicObstacles();
    Map<String, Point> boundaries = map.getConfirmedBoundariesPositions();

    double score = 1.0;

    // Boundary distance scoring
    for (Map.Entry<String, Point> boundary : boundaries.entrySet()) {
      Point boundaryPos = boundary.getValue();
      double distanceToBoundary = getDistance(nextPos, boundaryPos);

      // Penalize getting too close to boundaries
      if (distanceToBoundary < BOUNDARY_SAFETY_MARGIN) {
        score *= (distanceToBoundary / BOUNDARY_SAFETY_MARGIN);
      }
    }

    // Agent proximity scoring
    for (Map.Entry<Point, ObstacleInfo> entry : dynamicObstacles.entrySet()) {
      Point agentPos = entry.getKey();
      double currentDistance = getDistance(currentPos, agentPos);
      double nextDistance = getDistance(nextPos, agentPos);

      // Penalize moving closer to agents near boundaries
      if (nextDistance < currentDistance) {
        score *= 0.7;
      }
    }

    // Check for potential agent-boundary traps
    if (couldCreateTrap(nextPos, dynamicObstacles, boundaries)) {
      score *= 0.5;
    }

    return score;
  }

  private boolean couldCreateTrap(
    Point position,
    Map<Point, ObstacleInfo> dynamicObstacles,
    Map<String, Point> boundaries
  ) {
    // Count nearby boundaries and agents
    int nearbyBoundaries = 0;
    int nearbyAgents = 0;

    for (Point boundaryPos : boundaries.values()) {
      if (getDistance(position, boundaryPos) <= BOUNDARY_SAFETY_MARGIN) {
        nearbyBoundaries++;
      }
    }

    for (Point agentPos : dynamicObstacles.keySet()) {
      if (getDistance(position, agentPos) <= BOUNDARY_SAFETY_MARGIN) {
        nearbyAgents++;
      }
    }

    // Position might create a trap if too many constraints
    return nearbyBoundaries >= 2 && nearbyAgents > 0;
  }

  private double getDistance(Point p1, Point p2) {
    return Math.sqrt(Math.pow(p2.x - p1.x, 2) + Math.pow(p2.y - p1.y, 2));
  }

  private List<String> handleStuckNearBoundary(
    String agentId,
    List<String> availableDirections,
    LocalMap map,
    Point currentPos,
    Point previousPos,
    String previousMove,
    Map<String, Point> boundaries
  ) {
    // If stuck near boundary, prioritize moving away from it
    String boundaryEscapeDirection = getBoundaryEscapeDirection(
      currentPos,
      boundaries
    );
    if (
      boundaryEscapeDirection != null &&
      availableDirections.contains(boundaryEscapeDirection)
    ) {
      return Collections.singletonList(boundaryEscapeDirection);
    }

    // Fallback to original stuck handling
    List<String> fallbackDirections = new ArrayList<>(availableDirections);
    if (previousMove != null) {
      fallbackDirections.remove(previousMove);
    }

    return fallbackDirections.isEmpty()
      ? availableDirections
      : fallbackDirections;
  }

  private String getBoundaryEscapeDirection(
    Point position,
    Map<String, Point> boundaries
  ) {
    // Find closest boundary and return opposite direction
    String closestBoundary = null;
    double minDistance = Double.MAX_VALUE;

    for (Map.Entry<String, Point> entry : boundaries.entrySet()) {
      double distance = getDistance(position, entry.getValue());
      if (distance < minDistance) {
        minDistance = distance;
        closestBoundary = entry.getKey();
      }
    }

    if (closestBoundary == null) return null;

    // Return opposite direction of closest boundary
    switch (closestBoundary) {
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

  private boolean isDirectionSafe(
    String agentId,
    LocalMap map,
    Point currentPos,
    String direction,
    Map<String, Point> boundaries
  ) {
    try {
      Point nextPos = calculateNextPosition(currentPos, direction);
      boolean forbidden = map.isForbidden(nextPos);
      boolean hitsBoundary = wouldHitBoundary(
        agentId,
        map,
        currentPos,
        direction
      );

      if (DEBUG) {
        logger.info(
          String.format(
            "[Agent %s] Checking direction %s to %s - Forbidden: %b, Hits Boundary: %b",
            agentId,
            direction,
            nextPos,
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
          direction,
          e.getMessage()
        )
      );
      return false;
    }
  }
}
