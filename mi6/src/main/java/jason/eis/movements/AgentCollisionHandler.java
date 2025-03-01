package jason.eis.movements;

import jason.eis.LocalMap;
import jason.eis.LocalMap.ObstacleInfo;
import jason.eis.Point;
import java.util.*;
import java.util.concurrent.ConcurrentHashMap;
import java.util.logging.Logger;
import java.util.stream.Collectors;

public class AgentCollisionHandler {
  private static final Logger logger = Logger.getLogger(
    AgentCollisionHandler.class.getName()
  );

  // Simplified constants
  private static final int STUCK_THRESHOLD = 3;
  private static final int MEMORY_SIZE = 3;
  private static final int AWARENESS_DISTANCE = 2; // Fixed at 2 cells
  private static final int OSCILLATION_THRESHOLD = 2;

  private final Map<String, AgentState> agentStates = new ConcurrentHashMap<>();
  private final Random random = new Random();

  public static class AgentState {
    Point lastPosition;
    int stuckCount = 0;
    List<Point> moveHistory = new ArrayList<>();
    List<String> directionHistory = new ArrayList<>();
    Set<String> failedDirections = new HashSet<>();
    private long lastUpdateTime;

    void recordMove(Point position, String direction) {
      lastUpdateTime = System.currentTimeMillis();

      if (moveHistory.size() >= MEMORY_SIZE) {
        moveHistory.remove(0);
        directionHistory.remove(0);
      }
      moveHistory.add(position);
      directionHistory.add(direction);

      logger.severe(
        String.format(
          "Agent Movement - Position: %s, Direction: %s",
          position,
          direction
        )
      );
      logger.severe(String.format("Movement History: %s", moveHistory));
      logger.severe(String.format("Direction History: %s", directionHistory));

      if (isOscillating()) {
        logger.warning(
          String.format(
            "OSCILLATION DETECTED at %s with pattern: %s",
            position,
            directionHistory
          )
        );
      }

      if (lastPosition != null && lastPosition.equals(position)) {
        stuckCount++;
        if (direction != null) {
          failedDirections.add(direction);
          logger.severe(
            String.format(
              "Failed movement in direction: %s, Total failed: %s",
              direction,
              failedDirections
            )
          );
        }
        logger.severe(
          String.format(
            "STUCK DETECTED at %s for %d moves. Failed directions: %s",
            position,
            stuckCount,
            failedDirections
          )
        );
      } else {
        if (stuckCount > 0) {
          logger.severe(
            String.format(
              "UNSTUCK from position %s after %d moves",
              lastPosition,
              stuckCount
            )
          );
        }
        stuckCount = 0;
        failedDirections.clear();
      }

      lastPosition = position;
    }

    boolean isStuck() {
      return stuckCount >= STUCK_THRESHOLD;
    }

    boolean isOscillating() {
      if (moveHistory.size() < MEMORY_SIZE) return false;
      // Check if we're repeating the same positions
      return new HashSet<>(moveHistory).size() <= 2;
    }

    String getLastDirection() {
      return directionHistory.isEmpty()
        ? null
        : directionHistory.get(directionHistory.size() - 1);
    }

    boolean isStale() {
      return System.currentTimeMillis() - lastUpdateTime > 5000; // 5 second timeout
    }
  }

  public void recordMove(String agName, Point position, String direction) {
    AgentState state = agentStates.computeIfAbsent(
      agName,
      k -> new AgentState()
    );
    state.recordMove(position, direction);
  }

  public void recordSuccessfulMove(String agentName, Point newPosition) {
    recordMove(agentName, newPosition, null);
  }

  public void recordFailedMove(
    String agName,
    Point position,
    String attemptedDirection
  ) {
    logger.severe(
      String.format(
        "Agent %s: Failed move to %s, direction: %s",
        agName,
        position,
        attemptedDirection
      )
    );

    Point attemptedPos = calculateNextPosition(position, attemptedDirection);
    logger.severe(
      String.format(
        "Move failure analysis - Attempted position: %s",
        attemptedPos
      )
    );

    AgentState state = agentStates.computeIfAbsent(
      agName,
      k -> new AgentState()
    );
    state.recordMove(position, attemptedDirection);
  }

  public String resolveCollision(
    String agName,
    Point currentPos,
    Point intendedMove,
    LocalMap map,
    boolean hasBlock
  ) {
    AgentState state = agentStates.get(agName);
    if (state == null) return null;

    List<String> availableDirections = getAvailableDirections(currentPos, map);
    if (availableDirections.isEmpty()) {
      logger.warning(
        String.format("Agent %s: NO AVAILABLE MOVES at %s", agName, currentPos)
      );
      return null;
    }

    // Log the current situation
    logger.warning(
      String.format(
        "\nAgent %s: COLLISION RESOLUTION NEEDED at %s (has block: %b)",
        agName,
        currentPos,
        hasBlock
      )
    );
    logger.warning(
      String.format("Available directions: %s", availableDirections)
    );
    logger.warning(String.format("Intended move: %s", intendedMove));

    Map<Point, ObstacleInfo> nearbyAgents = getNearbyAgents(currentPos, map);
    logger.warning(String.format("Nearby agents: %s", nearbyAgents.keySet()));

    // Handle stuck situation first
    if (state.isStuck() || state.isOscillating()) {
      logger.warning(
        String.format(
          "Agent %s: ATTEMPTING RECOVERY from stuck/oscillation",
          agName
        )
      );
      String recoveryMove = getRecoveryMove(
        state,
        currentPos,
        availableDirections,
        map
      );
      if (recoveryMove != null) {
        logger.warning(String.format("Recovery move chosen: %s", recoveryMove));
        return recoveryMove;
      }
    }

    // Try to follow intended direction if safe
    if (intendedMove != null) {
      String intendedDir = getDirectionFromPoints(currentPos, intendedMove);
      if (intendedDir != null && availableDirections.contains(intendedDir)) {
        Point nextPos = calculateNextPosition(currentPos, intendedDir);
        if (!hasImmediateCollision(nextPos, map)) {
          logger.info(
            String.format("Following intended direction: %s", intendedDir)
          );
          return intendedDir;
        }
      }
    }

    // Choose safest available direction
    String safeDir = findSafestDirection(currentPos, availableDirections, map);
    logger.warning(String.format("Chose safest direction: %s", safeDir));
    return safeDir;
  }

  private String getRecoveryMove(
    AgentState state,
    Point currentPos,
    List<String> availableDirections,
    LocalMap map
  ) {
    String lastDir = state.getLastDirection();
    logger.warning(
      String.format(
        "Recovery attempt - Last direction: %s, Failed directions: %s",
        lastDir,
        state.failedDirections
      )
    );

    if (lastDir != null) {
      String oppositeDir = getOppositeDirection(lastDir);
      Point nextPos = calculateNextPosition(currentPos, oppositeDir);
      boolean isOppositeAvailable = availableDirections.contains(oppositeDir);
      boolean isOppositeCollision = hasImmediateCollision(nextPos, map);

      logger.warning(
        String.format(
          "Opposite direction %s - Available: %b, Will collide: %b",
          oppositeDir,
          isOppositeAvailable,
          isOppositeCollision
        )
      );

      if (isOppositeAvailable && !isOppositeCollision) {
        return oppositeDir;
      }
    }

    List<String> safeDirections = availableDirections
      .stream()
      .filter(dir -> !state.failedDirections.contains(dir))
      .collect(Collectors.toList());

    logger.warning(
      String.format("Safe directions available: %s", safeDirections)
    );

    if (!safeDirections.isEmpty()) {
      String chosen = safeDirections.get(random.nextInt(safeDirections.size()));
      logger.warning(String.format("Chose random safe direction: %s", chosen));
      return chosen;
    }

    String fallback = availableDirections.get(
      random.nextInt(availableDirections.size())
    );
    logger.warning(
      String.format("No safe directions, falling back to random: %s", fallback)
    );
    return fallback;
  }

  private Map<Point, ObstacleInfo> getNearbyAgents(Point pos, LocalMap map) {
    Map<Point, ObstacleInfo> nearby = map
      .getDynamicObstacles()
      .entrySet()
      .stream()
      .filter(e -> getManhattanDistance(pos, e.getKey()) <= AWARENESS_DISTANCE)
      .collect(Collectors.toMap(Map.Entry::getKey, Map.Entry::getValue));

    logger.warning(
      String.format(
        "Nearby agents around %s (distance %d): %s",
        pos,
        AWARENESS_DISTANCE,
        nearby.keySet()
      )
    );
    return nearby;
  }

  private boolean hasImmediateCollision(Point pos, LocalMap map) {
    return !getNearbyAgents(pos, map).isEmpty();
  }

  private String findSafestDirection(
    Point currentPos,
    List<String> availableDirections,
    LocalMap map
  ) {
    String bestDir = null;
    int minNearbyAgents = Integer.MAX_VALUE;

    for (String dir : availableDirections) {
      Point nextPos = calculateNextPosition(currentPos, dir);
      int nearbyAgents = getNearbyAgents(nextPos, map).size();

      if (nearbyAgents < minNearbyAgents) {
        minNearbyAgents = nearbyAgents;
        bestDir = dir;
      }
    }
    return bestDir != null ? bestDir : availableDirections.get(0);
  }

  // Helper methods
  private List<String> getAvailableDirections(Point currentPos, LocalMap map) {
    return Arrays
      .asList("n", "s", "e", "w")
      .stream()
      .filter(
        dir -> {
          Point next = calculateNextPosition(currentPos, dir);
          return !map.hasObstacle(next) && !map.isOutOfBounds(next);
        }
      )
      .collect(Collectors.toList());
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

  private String getDirectionFromPoints(Point from, Point to) {
    if (to.x > from.x) return "e";
    if (to.x < from.x) return "w";
    if (to.y > from.y) return "s";
    if (to.y < from.y) return "n";
    return null;
  }

  private String getOppositeDirection(String direction) {
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

  private int getManhattanDistance(Point p1, Point p2) {
    return Math.abs(p1.x - p2.x) + Math.abs(p1.y - p2.y);
  }

  private boolean isNearBoundary(Point pos, LocalMap map) {
    boolean isBoundary = map.isOutOfBounds(pos);
    if (isBoundary) {
      logger.severe(String.format("Boundary detected at position: %s", pos));
    }
    return isBoundary;
  }
}
