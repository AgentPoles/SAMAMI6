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

    // Get the planned move
    String plannedMove = state.getCurrentStep();
    if (plannedMove == null) return null;

    // Check for immediate threats and handle them
    if (hasImmediateThreat(currentPos, map)) {
      if (DEBUG) logger.info(
        String.format(
          "[%s] Immediate threat detected, using collision handler",
          agName
        )
      );
      return handleImmediateThreat(agName, state, currentPos, plannedMove, map);
    }

    // Look ahead for potential obstacles
    if (
      isPathSegmentBlocked(currentPos, state.getNextSteps(LOOKAHEAD_STEPS), map)
    ) {
      if (DEBUG) logger.info(
        String.format(
          "[%s] Path segment blocked, attempting resolution",
          agName
        )
      );
      return handleBlockedPath(agName, state, currentPos, map);
    }

    // Execute planned move if path is clear
    state.currentPathIndex++;
    return plannedMove;
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

  private String calculateIntendedMove(MovementState state, LocalMap map) {
    if (state.needsNewPath()) {
      Search.PathResult path = search.findPath(
        map.getCurrentPosition(),
        state.targetPosition,
        map,
        state.targetType
      );

      if (path.success) {
        state.plannedPath = path.directions;
        state.pathCalculatedTime = System.currentTimeMillis();
        state.currentPathIndex = 0;
      } else {
        return null;
      }
    }

    if (
      state.plannedPath.isEmpty() ||
      state.currentPathIndex >= state.plannedPath.size()
    ) {
      return null;
    }

    return state.plannedPath.get(state.currentPathIndex);
  }

  private String getNextSafeMove(String plannedDirection, LocalMap map) {
    if (!MovementUtils.wouldHitBoundary(map, plannedDirection)) {
      return plannedDirection;
    }
    // Need to replan path
    return null;
  }

  public String getNextMove(
    String agName,
    LocalMap map,
    Point target,
    boolean hasBlock
  ) {
    Point currentPos = map.getCurrentPosition();

    // Get or calculate path
    Deque<String> path = getOrCalculatePath(agName, currentPos, target, map);
    if (path == null || path.isEmpty()) {
      return null;
    }

    String intendedDirection = path.peek();

    // Use collision handler to resolve any potential conflicts
    String resolvedDirection = collisionHandler.resolveCollision(
      agName,
      currentPos,
      intendedDirection,
      map,
      hasBlock
    );

    // If the resolved direction matches intended, remove it from the path
    if (resolvedDirection.equals(intendedDirection)) {
      path.poll();
    }
    // If we got a different direction or skip, we'll keep the path but try again next turn

    return resolvedDirection;
  }

  private Deque<String> getOrCalculatePath(
    String agName,
    Point current,
    Point target,
    LocalMap map
  ) {
    Deque<String> existingPath = plannedPaths.get(agName);
    MovementState state = agentStates.get(agName);

    // Recalculate path if none exists or target changed
    if (
      existingPath == null ||
      existingPath.isEmpty() ||
      !willReachTarget(current, existingPath, target)
    ) {
      Search.PathResult result = calculatePath(
        map,
        current,
        target,
        state.targetType
      );
      if (result != null && result.success) {
        return new ArrayDeque<>(result.directions);
      }
    }
    return existingPath != null ? existingPath : new ArrayDeque<>();
  }

  /**
   * Checks if there are any immediate threats (agents very close by)
   */
  private boolean hasImmediateThreat(Point currentPos, LocalMap map) {
    return map
      .getDynamicObstacles()
      .values()
      .stream()
      .anyMatch(
        obstacle -> {
          Point obstaclePos = obstacle.getPosition();
          return (
            getManhattanDistance(currentPos, obstaclePos) <= CRITICAL_DISTANCE
          );
        }
      );
  }

  /**
   * Handles immediate threats using the collision handler
   */
  private String handleImmediateThreat(
    String agName,
    MovementState state,
    Point currentPos,
    String plannedMove,
    LocalMap map
  ) {
    String resolvedMove = collisionHandler.resolveCollision(
      agName,
      currentPos,
      plannedMove,
      map,
      false
    );

    // If we got a different move, increment deviation attempts
    if (!resolvedMove.equals(plannedMove)) {
      state.deviationAttempts++;
      state.isDeviating = true;
    }

    return resolvedMove;
  }

  /**
   * Checks if a sequence of planned moves is blocked by dynamic obstacles
   */
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

  /**
   * Handles a blocked path segment with progressive response levels
   */
  private String handleBlockedPath(
    String agName,
    MovementState state,
    Point currentPos,
    LocalMap map
  ) {
    // Level 4: If we've deviated too many times, recalculate entire path
    if (state.deviationAttempts >= MAX_DEVIATION_ATTEMPTS) {
      if (DEBUG) logger.info(
        String.format("[%s] Max deviations reached, recalculating path", agName)
      );
      return recalculateEntirePath(agName, state, currentPos, map);
    }

    // Level 3: Try to find alternative path segment
    List<String> alternativeSegment = findAlternativeSegment(
      currentPos,
      state.targetPosition,
      state.getNextSteps(LOOKAHEAD_STEPS),
      map
    );

    if (alternativeSegment != null && !alternativeSegment.isEmpty()) {
      // Replace current path segment with alternative
      state.plannedPath = new ArrayList<>(alternativeSegment);
      state.currentPathIndex = 0;
      state.isDeviating = true;
      state.deviationAttempts++;
      return state.getCurrentStep();
    }

    // Level 2: If no alternative segment found, use collision handler for one step
    return handleImmediateThreat(
      agName,
      state,
      currentPos,
      state.getCurrentStep(),
      map
    );
  }

  /**
   * Attempts to find an alternative path segment that avoids obstacles
   */
  private List<String> findAlternativeSegment(
    Point start,
    Point target,
    List<String> originalSegment,
    LocalMap map
  ) {
    // Try to find a path that rejoins the original path
    Point rejoinPoint = findRejoinPoint(start, originalSegment, map);
    if (rejoinPoint != null) {
      Search.PathResult altPath = search.findPath(
        start,
        rejoinPoint,
        map,
        null
      );
      if (altPath != null && altPath.success) {
        return altPath.directions;
      }
    }
    return null;
  }

  /**
   * Finds a suitable point to rejoin the original path
   */
  private Point findRejoinPoint(
    Point start,
    List<String> remainingPath,
    LocalMap map
  ) {
    Point current = new Point(start.x, start.y);
    for (int i = 0; i < remainingPath.size(); i++) {
      current = calculateNextPosition(current, remainingPath.get(i));
      if (!isDynamicallyBlocked(current, map)) {
        return current;
      }
    }
    return null;
  }

  /**
   * Recalculates the entire path when other strategies fail
   */
  private String recalculateEntirePath(
    String agName,
    MovementState state,
    Point currentPos,
    LocalMap map
  ) {
    Search.PathResult newPath = search.findPath(
      currentPos,
      state.targetPosition,
      map,
      state.targetType
    );

    if (newPath != null && newPath.success) {
      state.setNewPath(newPath.directions);
      return state.getCurrentStep();
    }

    // If recalculation fails, return null to trigger fallback behavior
    return null;
  }

  /**
   * Checks if a position is blocked by dynamic obstacles
   */
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

  /**
   * Checks if the current path will reach the target
   */
  private boolean willReachTarget(
    Point current,
    Deque<String> path,
    Point target
  ) {
    if (path == null || path.isEmpty()) return false;
    Point finalPos = current;
    for (String move : path) {
      finalPos = calculateNextPosition(finalPos, move);
    }
    return finalPos.equals(target);
  }
}
