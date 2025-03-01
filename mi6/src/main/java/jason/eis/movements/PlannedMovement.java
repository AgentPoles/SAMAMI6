package jason.eis.movements;

import jason.eis.LocalMap;
import jason.eis.LocalMap.ObstacleInfo;
import jason.eis.MI6Model;
import jason.eis.Point;
import java.util.*;
import java.util.logging.Logger;
import java.util.stream.Collectors;

public class PlannedMovement implements MovementStrategy {
  private static final Logger logger = Logger.getLogger(
    PlannedMovement.class.getName()
  );
  private static final boolean DEBUG = true;
  private static final int MAX_SEARCH_RANGE = 50; // Configurable max range
  private static final int PATH_TIMEOUT = 5000; // 5 seconds before recalculating path
  private static final int MAX_TARGETS_TO_CHECK = 5;

  // Dynamic obstacle handling constants
  private static final int CRITICAL_DISTANCE = 2; // Distance to react to agents
  private static final int LOOKAHEAD_STEPS = 3; // Number of steps to check ahead
  private static final int MAX_DEVIATION_ATTEMPTS = 3; // Max attempts to deviate before full recalc
  private static final double COLLISION_RISK_THRESHOLD = 0.7; // Risk threshold for path deviation

  private final Search search;
  private final Map<String, MovementState> agentStates;
  private final AgentCollisionHandler collisionHandler;
  private final Map<String, Deque<String>> plannedPaths = new HashMap<>();

  public PlannedMovement() {
    this.search = new Search();
    this.agentStates = new HashMap<>();
    this.collisionHandler = new AgentCollisionHandler();
  }

  private static class MovementState {
    Point targetPosition;
    Search.TargetType targetType;
    List<String> plannedPath;
    List<String> originalPath; // Keep original path for deviation checks
    long pathCalculatedTime;
    int currentPathIndex;
    int deviationAttempts; // Track number of deviation attempts
    boolean isDeviating; // Flag if currently following a deviation

    MovementState(Point target, Search.TargetType type) {
      this.targetPosition = target;
      this.targetType = type;
      this.plannedPath = new ArrayList<>();
      this.originalPath = new ArrayList<>();
      this.pathCalculatedTime = 0;
      this.currentPathIndex = 0;
      this.deviationAttempts = 0;
      this.isDeviating = false;
    }

    void setNewPath(List<String> path) {
      this.plannedPath = new ArrayList<>(path);
      this.originalPath = new ArrayList<>(path);
      this.pathCalculatedTime = System.currentTimeMillis();
      this.currentPathIndex = 0;
      this.deviationAttempts = 0;
      this.isDeviating = false;
    }

    List<String> getNextSteps(int count) {
      int endIndex = Math.min(currentPathIndex + count, plannedPath.size());
      return plannedPath.subList(currentPathIndex, endIndex);
    }

    String getCurrentStep() {
      return currentPathIndex < plannedPath.size()
        ? plannedPath.get(currentPathIndex)
        : null;
    }

    boolean needsNewPath() {
      return (
        plannedPath.isEmpty() ||
        currentPathIndex >= plannedPath.size() ||
        System.currentTimeMillis() - pathCalculatedTime > PATH_TIMEOUT
      );
    }

    @Override
    public String toString() {
      return String.format(
        "MovementState{target=%s, type=%s, pathSize=%d, index=%d, age=%dms}",
        targetPosition,
        targetType,
        plannedPath.size(),
        currentPathIndex,
        System.currentTimeMillis() - pathCalculatedTime
      );
    }
  }

  public Point findNearestTarget(
    LocalMap map,
    Point currentPos,
    Search.TargetType targetType
  ) {
    if (map == null || currentPos == null || targetType == null) {
      logger.warning("Invalid parameters in findNearestTarget");
      return null;
    }

    List<Point> targets = getTargetsOfType(map, targetType);
    if (targets.isEmpty()) {
      if (DEBUG) logger.info("No targets found for type: " + targetType);
      return null;
    }

    // Sort targets by Manhattan distance
    targets.sort(
      (a, b) -> {
        int distA = getManhattanDistance(currentPos, a);
        int distB = getManhattanDistance(currentPos, b);
        return Integer.compare(distA, distB);
      }
    );

    // Check closest targets for best path
    List<Point> nearestTargets = targets.subList(
      0,
      Math.min(MAX_TARGETS_TO_CHECK, targets.size())
    );
    return findBestTarget(currentPos, nearestTargets, map, targetType);
  }

  private Point findBestTarget(
    Point currentPos,
    List<Point> targets,
    LocalMap map,
    Search.TargetType targetType
  ) {
    Point bestTarget = null;
    int minDistance = Integer.MAX_VALUE;

    for (Point target : targets) {
      try {
        int manhattanDist = getManhattanDistance(currentPos, target);
        if (manhattanDist >= minDistance || manhattanDist > MAX_SEARCH_RANGE) {
          continue;
        }

        Search.PathResult path = search.findPath(
          currentPos,
          target,
          map,
          targetType
        );
        if (path != null && path.success && path.points.size() < minDistance) {
          minDistance = path.points.size();
          bestTarget = target;
          if (DEBUG) {
            logger.info(
              String.format(
                "New best target found at %s with path length %d",
                bestTarget,
                minDistance
              )
            );
          }
        }
      } catch (Exception e) {
        logger.warning(
          "Error evaluating target " + target + ": " + e.getMessage()
        );
      }
    }
    return bestTarget;
  }

  @Override
  public String getNextMove(String agName, LocalMap map) {
    MovementState state = agentStates.get(agName);
    if (state == null || map == null) return null;

    Point currentPos = map.getCurrentPosition();
    if (currentPos == null) return null;

    // Get or calculate the planned move
    String plannedMove = getPlannedMove(state, currentPos, map);
    if (plannedMove == null) return null;

    // Convert planned move direction to target Point
    Point targetPos = calculateNextPosition(currentPos, plannedMove);

    // Let collision handler handle both dynamic obstacles and boundaries
    String resolvedMove = collisionHandler.resolveCollision(
      agName,
      currentPos,
      targetPos,
      map,
      false
    );

    if (resolvedMove != null) {
      // If we got a different move than planned, handle deviation
      if (!resolvedMove.equals(plannedMove)) {
        handleDeviation(state);
      } else {
        state.currentPathIndex++;
      }
    }

    return resolvedMove;
  }

  public String getNextMove(String agName) {
    LocalMap map = MI6Model.getInstance().getAgentMap(agName);
    return getNextMove(agName, map);
  }

  public void setTarget(String agName, Point target, Search.TargetType type) {
    try {
      if (agName == null) {
        logger.warning("Null agent name in setTarget");
        return;
      }
      MovementState state = new MovementState(target, type);
      agentStates.put(agName, state);
    } catch (Exception e) {
      logger.warning("Error in setTarget: " + e.getMessage());
    }
  }

  public void clearTarget(String agName) {
    try {
      if (agName != null) {
        agentStates.remove(agName);
      }
    } catch (Exception e) {
      logger.warning("Error clearing target: " + e.getMessage());
    }
  }

  public boolean hasTarget(String agName) {
    try {
      return (
        agName != null &&
        agentStates.containsKey(agName) &&
        agentStates.get(agName).targetPosition != null
      );
    } catch (Exception e) {
      logger.warning("Error checking target: " + e.getMessage());
      return false;
    }
  }

  public void moveSucceeded(String agName) {
    try {
      MovementState state = agentStates.get(agName);
      if (state != null) {
        state.currentPathIndex++;
        // Record successful move in collision handler
        LocalMap map = MI6Model.getInstance().getAgentMap(agName);
        if (map != null) {
          collisionHandler.recordSuccessfulMove(
            agName,
            map.getCurrentPosition()
          );
        }
      }
    } catch (Exception e) {
      logger.warning("Error in moveSucceeded: " + e.getMessage());
    }
  }

  public void moveFailed(String agName) {
    try {
      if (agName != null) {
        agentStates.remove(agName);
      }
    } catch (Exception e) {
      logger.warning("Error in moveFailed: " + e.getMessage());
    }
  }

  private int getManhattanDistance(Point p1, Point p2) {
    if (p1 == null || p2 == null) {
      throw new IllegalArgumentException(
        "Cannot calculate distance with null points"
      );
    }
    return Math.abs(p1.x - p2.x) + Math.abs(p1.y - p2.y);
  }

  private List<Point> getTargetsOfType(
    LocalMap map,
    Search.TargetType targetType
  ) {
    try {
      if (map == null || targetType == null) {
        logger.warning("Invalid parameters in getTargetsOfType");
        return new ArrayList<>();
      }

      switch (targetType) {
        case DISPENSER:
          return new ArrayList<>(map.getDispensers());
        case BLOCK:
          return new ArrayList<>(map.getBlocks());
        case GOAL:
          return new ArrayList<>(map.getGoals());
        default:
          logger.warning("Unknown target type: " + targetType);
          return new ArrayList<>();
      }
    } catch (Exception e) {
      logger.warning("Error getting targets: " + e.getMessage());
      return new ArrayList<>();
    }
  }

  public Search.PathResult calculatePath(
    LocalMap map,
    Point start,
    Point goal,
    Search.TargetType targetType
  ) {
    return search.findPath(start, goal, map, targetType);
  }

  public Search.PathResult getFullPathResult(String agName, LocalMap map) {
    MovementState state = agentStates.get(agName);
    if (state == null || state.targetPosition == null) {
      return new Search.PathResult(new ArrayList<>(), new ArrayList<>(), false);
    }

    return search.findPath(
      map.getCurrentPosition(),
      state.targetPosition,
      map,
      state.targetType
    );
  }

  private String getPlannedMove(
    MovementState state,
    Point currentPos,
    LocalMap map
  ) {
    // Recalculate path if needed
    if (state.needsNewPath()) {
      Search.PathResult path = search.findPath(
        currentPos,
        state.targetPosition,
        map,
        state.targetType
      );

      if (path != null && path.success) {
        state.setNewPath(path.directions);
      } else {
        return null;
      }
    }

    return state.getCurrentStep();
  }

  private void handleDeviation(MovementState state) {
    state.deviationAttempts++;
    state.isDeviating = true;

    // If we've deviated too many times, force path recalculation
    if (state.deviationAttempts >= MAX_DEVIATION_ATTEMPTS) {
      state.plannedPath.clear(); // This will trigger recalculation on next turn
    }
  }

  private boolean isPathSegmentBlocked(
    Point start,
    List<String> moves,
    LocalMap map
  ) {
    Point current = new Point(start.x, start.y);
    for (String move : moves) {
      current = calculateNextPosition(current, move);
      if (isDynamicallyBlocked(current, map)) {
        return true;
      }
    }
    return false;
  }

  private boolean isDynamicallyBlocked(Point pos, LocalMap map) {
    return map
      .getDynamicObstacles()
      .values()
      .stream()
      .anyMatch(
        obstacle -> {
          Point obstaclePos = obstacle.getPosition();
          return (
            obstaclePos.equals(pos) ||
            (
              getManhattanDistance(pos, obstaclePos) <= 1 &&
              calculateCollisionRisk(pos, obstacle) > COLLISION_RISK_THRESHOLD
            )
          );
        }
      );
  }

  /**
   * Calculates risk of collision with a dynamic obstacle
   */
  private double calculateCollisionRisk(Point pos, ObstacleInfo obstacle) {
    // Basic risk calculation based on distance and movement
    Point obstaclePos = obstacle.getPosition();
    int distance = getManhattanDistance(pos, obstaclePos);

    // Higher risk for obstacles carrying blocks
    double baseRisk = obstacle.hasBlock() ? 0.8 : 0.6;

    // Risk decreases with distance
    return baseRisk / (distance + 1);
  }

  /**
   * Calculates the next position based on current position and direction
   */
  private Point calculateNextPosition(Point current, String direction) {
    if (direction == null) return current;
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
