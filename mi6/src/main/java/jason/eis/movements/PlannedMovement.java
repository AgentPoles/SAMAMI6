package jason.eis.movements;

import jason.eis.LocalMap;
import jason.eis.LocalMap.ObstacleInfo;
import jason.eis.MI6Model;
import jason.eis.Point;
import jason.eis.movements.collision.CollisionResolution;
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

  private final Random random = new Random();
  private final BoundaryManager boundaryManager = new BoundaryManager();
  private final ObstacleManager obstacleManager = new ObstacleManager();

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
    String agentName;
    Point targetPosition;
    Search.TargetType targetType;
    List<String> plannedPath;
    List<String> originalPath;
    long pathCalculatedTime;
    int currentPathIndex;
    int deviationAttempts;
    boolean isDeviating;
    List<Point> recentPositions;

    MovementState(String agentName, Point target, Search.TargetType type) {
      this.agentName = agentName;
      this.targetPosition = target;
      this.targetType = type;
      this.plannedPath = new ArrayList<>();
      this.originalPath = new ArrayList<>();
      this.pathCalculatedTime = 0;
      this.currentPathIndex = 0;
      this.deviationAttempts = 0;
      this.isDeviating = false;
      this.recentPositions = new ArrayList<>();
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

    boolean isOscillating() {
      if (recentPositions.size() < 4) return false;

      // Check last 4 positions for oscillation pattern
      int last = recentPositions.size() - 1;
      return (
        recentPositions.get(last).equals(recentPositions.get(last - 2)) &&
        recentPositions.get(last - 1).equals(recentPositions.get(last - 3))
      );
    }

    void updatePosition(Point pos) {
      recentPositions.add(pos);
      if (recentPositions.size() > 4) {
        recentPositions.remove(0);
      }
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
    // Default to size 1 and no block direction
    return findNearestTarget(map, currentPos, targetType, 1, null);
  }

  public Point findNearestTarget(
    LocalMap map,
    Point currentPos,
    Search.TargetType targetType,
    int size,
    String blockDirection
  ) {
    try {
      if (!validateTargetSearchInputs(map, currentPos, targetType)) {
        return null;
      }

      List<Point> targets = getTargetsOfType(map, targetType);
      if (targets.isEmpty()) {
        debug("No targets found for type: %s", targetType);
        return null;
      }

      // Sort and filter targets safely
      List<Point> sortedTargets = getSortedTargets(targets, currentPos);
      List<Point> nearestTargets = getNearestTargets(sortedTargets);

      return findBestTarget(
        currentPos,
        nearestTargets,
        map,
        targetType,
        size,
        blockDirection
      );
    } catch (Exception e) {
      logger.severe(
        String.format("Critical error in findNearestTarget: %s", e.getMessage())
      );
      return null;
    }
  }

  private boolean validateTargetSearchInputs(
    LocalMap map,
    Point currentPos,
    Search.TargetType targetType
  ) {
    if (map == null) {
      logger.warning("Map is null in target search");
      return false;
    }
    if (currentPos == null) {
      logger.warning("Current position is null in target search");
      return false;
    }
    if (targetType == null) {
      logger.warning("Target type is null in target search");
      return false;
    }
    return true;
  }

  private List<Point> getSortedTargets(List<Point> targets, Point currentPos) {
    try {
      return targets
        .stream()
        .filter(Objects::nonNull)
        .sorted(
          (a, b) -> {
            try {
              return Integer.compare(
                getManhattanDistance(currentPos, a),
                getManhattanDistance(currentPos, b)
              );
            } catch (Exception e) {
              logger.warning("Error comparing distances: " + e.getMessage());
              return 0;
            }
          }
        )
        .collect(Collectors.toList());
    } catch (Exception e) {
      logger.warning("Error sorting targets: " + e.getMessage());
      return new ArrayList<>(targets);
    }
  }

  private List<Point> getNearestTargets(List<Point> sortedTargets) {
    try {
      return sortedTargets.subList(
        0,
        Math.min(MAX_TARGETS_TO_CHECK, sortedTargets.size())
      );
    } catch (Exception e) {
      logger.warning("Error getting nearest targets: " + e.getMessage());
      return sortedTargets;
    }
  }

  private Point findBestTarget(
    Point currentPos,
    List<Point> targets,
    LocalMap map,
    Search.TargetType targetType,
    int size,
    String blockDirection
  ) {
    try {
      Point bestTarget = null;
      int minDistance = Integer.MAX_VALUE;

      for (Point target : targets) {
        try {
          int manhattanDist = getManhattanDistance(currentPos, target);
          if (
            manhattanDist >= minDistance || manhattanDist > MAX_SEARCH_RANGE
          ) {
            continue;
          }

          Search.PathResult path = search.findPath(
            currentPos,
            target,
            map,
            targetType,
            size,
            blockDirection,
            Search.PathRecomputeReason.DEFAULT
          );

          if (path != null && path.success) {
            bestTarget = target;
            minDistance = manhattanDist;
          }
        } catch (Exception e) {
          logger.warning(
            String.format(
              "Error checking target %s: %s",
              target,
              e.getMessage()
            )
          );
        }
      }

      return bestTarget;
    } catch (Exception e) {
      logger.severe("Critical error in findBestTarget: " + e.getMessage());
      return null;
    }
  }

  private Search.PathResult searchPathSafely(
    Point start,
    Point goal,
    LocalMap map,
    Search.TargetType targetType,
    int size,
    String blockDirection
  ) {
    try {
      return search.findPath(
        start,
        goal,
        map,
        targetType,
        size,
        blockDirection,
        Search.PathRecomputeReason.DEFAULT
      );
    } catch (Exception e) {
      logger.warning(
        String.format(
          "Path finding error from %s to %s: %s",
          start,
          goal,
          e.getMessage()
        )
      );
      return null;
    }
  }

  private boolean isPathValid(Search.PathResult path) {
    return path != null && path.success && !path.points.isEmpty();
  }

  @Override
  public String getNextMove(String agName, LocalMap map) {
    // Call getNextDirection with default size 1 and no block
    return getNextDirection(agName, map, 1, null);
  }

  @Override
  public String getNextDirection(
    String agName,
    LocalMap map,
    int size,
    String blockDirection
  ) {
    try {
      if (agName == null || map == null) {
        logWarningf(
          "Invalid parameters in getNextDirection: agent=%s, map=%s",
          agName,
          map != null ? "valid" : "null"
        );
        return null;
      }

      MovementState state = agentStates.get(agName);
      if (state == null) {
        debug("No movement state for agent: %s", agName);
        return null;
      }

      Point currentPos = map.getCurrentPosition();
      if (currentPos == null) {
        logWarningf("[%s] Current position is null", agName);
        return null;
      }

      debug(
        "[%s] Processing movement at pos %s to target %s (type: %s)",
        agName,
        currentPos,
        state.targetPosition,
        state.targetType
      );

      // Get and validate available directions
      List<String> availableDirections = getValidDirections(
        agName,
        currentPos,
        map,
        size,
        blockDirection,
        state
      );
      if (availableDirections.isEmpty()) {
        debug(
          "[%s] No available directions at position %s",
          agName,
          currentPos
        );
        return null;
      }

      // Handle collisions first
      String collisionDirection = handleCollisions(
        agName,
        currentPos,
        state,
        map,
        size,
        blockDirection,
        availableDirections
      );
      if (collisionDirection != null) {
        debug(
          "[%s] Using collision resolution direction: %s",
          agName,
          collisionDirection
        );
        return collisionDirection;
      }

      // Check if we've reached the target
      if (hasReachedTarget(currentPos, state.targetPosition, size)) {
        debug("[%s] Reached target position %s", agName, state.targetPosition);
        clearTarget(agName);
        return null;
      }

      // Get planned move
      String plannedMove = getPlannedMove(
        state,
        currentPos,
        map,
        size,
        blockDirection
      );
      if (plannedMove != null && availableDirections.contains(plannedMove)) {
        debug("[%s] Using planned move: %s", agName, plannedMove);
        state.currentPathIndex++;
        return plannedMove;
      }

      // Fallback to best available direction
      String bestDirection = chooseBestDirection(
        availableDirections,
        currentPos,
        state.targetPosition,
        map
      );
      debug("[%s] Using best available direction: %s", agName, bestDirection);
      handleDeviation(
        state,
        bestDirection,
        currentPos,
        map,
        size,
        blockDirection
      );
      return bestDirection;
    } catch (Exception e) {
      logWarningf("Error in getNextDirection: %s", e.getMessage());
      return null;
    }
  }

  private boolean validateInputs(String agName, LocalMap map, int size) {
    if (agName == null) {
      logger.warning("Agent name is null");
      return false;
    }
    if (map == null) {
      logger.warning(String.format("[%s] Map is null", agName));
      return false;
    }
    if (size < 1) {
      logger.warning(String.format("[%s] Invalid size: %d", agName, size));
      return false;
    }
    return true;
  }

  private List<String> getValidDirections(
    String agName,
    Point currentPos,
    LocalMap map,
    int size,
    String blockDirection,
    MovementState state
  ) {
    try {
      List<String> directions = Arrays.asList("n", "s", "e", "w");

      // Apply boundary constraints
      directions =
        boundaryManager.filterDirections(
          agName,
          directions,
          map,
          currentPos,
          state.targetPosition,
          state.getCurrentStep()
        );

      // Apply obstacle constraints
      directions =
        obstacleManager.filterDirections(
          agName,
          directions,
          map,
          currentPos,
          size,
          blockDirection,
          state.targetPosition,
          state.getCurrentStep()
        );

      debug(
        "[%s] Valid directions after filtering: %s",
        agName,
        String.join(",", directions)
      );
      return directions;
    } catch (Exception e) {
      logger.warning(
        String.format(
          "[%s] Error getting valid directions: %s",
          agName,
          e.getMessage()
        )
      );
      return new ArrayList<>();
    }
  }

  private String handleCollisions(
    String agName,
    Point currentPos,
    MovementState state,
    LocalMap map,
    int size,
    String blockDirection,
    List<String> availableDirections
  ) {
    try {
      CollisionResolution resolution = collisionHandler.resolveCollision(
        agName,
        currentPos,
        state.getCurrentStep(),
        map,
        size,
        blockDirection,
        availableDirections
      );

      if (resolution != null) {
        String direction = resolution.getDirection();
        debug(
          "[%s] Collision resolved with direction: %s, reason: %s",
          agName,
          direction,
          resolution.getReason()
        );

        if ("STUCK".equals(resolution.getReason())) {
          state.plannedPath.clear();
          debug("[%s] Clearing planned path due to STUCK state", agName);
        }

        handleDeviation(
          state,
          direction,
          currentPos,
          map,
          size,
          blockDirection
        );
        return direction;
      }
      return null;
    } catch (Exception e) {
      logger.warning(
        String.format(
          "[%s] Error handling collisions: %s",
          agName,
          e.getMessage()
        )
      );
      return null;
    }
  }

  private boolean hasReachedTarget(
    Point currentPos,
    Point targetPos,
    int size
  ) {
    if (currentPos == null || targetPos == null) return false;

    // For size > 1, check if we're adjacent to the target
    int distance = getManhattanDistance(currentPos, targetPos);
    return size == 1 ? distance == 0 : distance <= 1;
  }

  private void debug(String format, Object... args) {
    if (DEBUG) {
      logger.fine(String.format(format, args));
    }
  }

  private String chooseBestDirection(
    List<String> availableDirections,
    Point currentPos,
    Point target,
    LocalMap map
  ) {
    String bestDir = null;
    double bestScore = Double.NEGATIVE_INFINITY;

    for (String direction : availableDirections) {
      Point nextPos = calculateNextPosition(currentPos, direction);
      double score = -getManhattanDistance(nextPos, target); // Negative because closer is better

      // Add some randomization to break ties
      score += random.nextDouble() * 0.1;

      if (score > bestScore) {
        bestScore = score;
        bestDir = direction;
      }
    }

    return bestDir != null ? bestDir : availableDirections.get(0);
  }

  private void handleDeviation(
    MovementState state,
    String deviationMove,
    Point currentPos,
    LocalMap map,
    int size,
    String blockDirection
  ) {
    state.deviationAttempts++;
    state.isDeviating = true;

    // Force path recalculation if we've deviated too many times
    if (state.deviationAttempts >= MAX_DEVIATION_ATTEMPTS) {
      state.plannedPath.clear();

      // Recalculate path starting from the deviation
      Point nextPos = calculateNextPosition(currentPos, deviationMove);
      Search.PathResult newPath = search.findPath(
        nextPos,
        state.targetPosition,
        map,
        state.targetType,
        size,
        blockDirection,
        Search.PathRecomputeReason.STUCK
      );

      if (newPath != null && newPath.success) {
        List<String> newDirections = new ArrayList<>();
        newDirections.add(deviationMove);
        newDirections.addAll(newPath.directions);
        state.setNewPath(newDirections);
      }
    }
  }

  public void setTarget(String agName, Point target, Search.TargetType type) {
    try {
      if (agName == null) {
        logger.warning("Null agent name in setTarget");
        return;
      }
      MovementState state = new MovementState(agName, target, type);
      agentStates.put(agName, state);
    } catch (Exception e) {
      logger.warning(formatLog("Error in setTarget: %s", e.getMessage()));
    }
  }

  private String formatLog(String format, Object... args) {
    return String.format(format, args);
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
      if (state == null) {
        debug("[%s] No state found for move success", agName);
        return;
      }

      state.currentPathIndex++;
      debug(
        "[%s] Move succeeded, new path index: %d",
        agName,
        state.currentPathIndex
      );
    } catch (Exception e) {
      logger.warning(
        String.format("[%s] Error in moveSucceeded: %s", agName, e.getMessage())
      );
    }
  }

  public void moveFailed(String agName) {
    try {
      if (agName == null) {
        logger.warning("Agent name is null in moveFailed");
        return;
      }

      agentStates.remove(agName);
      debug("[%s] Move failed, cleared movement state", agName);
    } catch (Exception e) {
      logger.warning(
        String.format("[%s] Error in moveFailed: %s", agName, e.getMessage())
      );
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
    Search.TargetType targetType,
    int size,
    String blockDirection
  ) {
    return search.findPath(
      start,
      goal,
      map,
      targetType,
      size,
      blockDirection,
      Search.PathRecomputeReason.DEFAULT
    );
  }

  public Search.PathResult getFullPathResult(
    String agName,
    LocalMap map,
    int size,
    String blockDirection
  ) {
    MovementState state = agentStates.get(agName);
    if (state == null || state.targetPosition == null) {
      return new Search.PathResult(new ArrayList<>(), new ArrayList<>(), false);
    }

    return search.findPath(
      map.getCurrentPosition(),
      state.targetPosition,
      map,
      state.targetType,
      size,
      blockDirection,
      Search.PathRecomputeReason.DEFAULT
    );
  }

  private String getPlannedMove(
    MovementState state,
    Point currentPos,
    LocalMap map,
    int size,
    String blockDirection
  ) {
    try {
      if (!validatePlannedMoveInputs(state, currentPos, map)) {
        debug("Invalid inputs for planned move");
        return null;
      }

      // Check if target still exists and is valid
      if (!isTargetStillValid(state, map)) {
        debug(
          "Target no longer valid: %s of type %s",
          state.targetPosition,
          state.targetType
        );
        clearTarget(state.agentName);
        return null;
      }

      // Handle deviation if needed
      if (state.isDeviating) {
        debug("Handling deviation for agent %s", state.agentName);
        return handleDeviation(state, currentPos, map, size, blockDirection);
      }

      // Use existing path if valid
      if (!state.needsNewPath() && state.getCurrentStep() != null) {
        String nextMove = state.getCurrentStep();
        if (isValidMove(currentPos, nextMove, map, size, blockDirection)) {
          debug("Using existing path, next move: %s", nextMove);
          state.currentPathIndex++;
          return nextMove;
        }
      }

      // Recompute path if needed
      debug("Recomputing path from %s to %s", currentPos, state.targetPosition);
      return recomputePath(state, currentPos, map, size, blockDirection);
    } catch (Exception e) {
      logger.severe(
        String.format("Critical error in getPlannedMove: %s", e.getMessage())
      );
      return null;
    }
  }

  private String handleDeviation(
    MovementState state,
    Point currentPos,
    LocalMap map,
    int size,
    String blockDirection
  ) {
    try {
      Search.PathResult newPath = search.findPath(
        currentPos,
        state.targetPosition,
        map,
        state.targetType,
        size,
        blockDirection,
        getRecomputeReason(state)
      );

      if (newPath == null || !newPath.success) {
        debug("Deviation path finding failed, clearing state");
        state.plannedPath.clear();
        return null;
      }

      state.setNewPath(newPath.directions);
      state.isDeviating = false;

      String nextMove = state.getCurrentStep();
      if (nextMove != null) {
        state.currentPathIndex++;
        return nextMove;
      }

      return null;
    } catch (Exception e) {
      logger.warning(
        String.format("Error handling deviation: %s", e.getMessage())
      );
      return null;
    }
  }

  private String recomputePath(
    MovementState state,
    Point currentPos,
    LocalMap map,
    int size,
    String blockDirection
  ) {
    try {
      Search.PathResult path = search.findPath(
        currentPos,
        state.targetPosition,
        map,
        state.targetType,
        size,
        blockDirection,
        Search.PathRecomputeReason.DEFAULT
      );

      if (path == null || !path.success) {
        debug("Path recomputation failed, clearing state");
        state.plannedPath.clear();
        return null;
      }

      state.setNewPath(path.directions);
      String nextMove = state.getCurrentStep();
      if (nextMove != null) {
        state.currentPathIndex++;
        return nextMove;
      }

      return null;
    } catch (Exception e) {
      logger.warning(
        String.format("Error recomputing path: %s", e.getMessage())
      );
      return null;
    }
  }

  private Search.PathRecomputeReason getRecomputeReason(MovementState state) {
    if (state.deviationAttempts >= MAX_DEVIATION_ATTEMPTS) {
      return Search.PathRecomputeReason.STUCK;
    }
    if (state.isOscillating()) {
      return Search.PathRecomputeReason.OSCILLATION;
    }
    if (System.currentTimeMillis() - state.pathCalculatedTime > PATH_TIMEOUT) {
      return Search.PathRecomputeReason.PATH_TIMEOUT;
    }
    return Search.PathRecomputeReason.DEFAULT;
  }

  private boolean validatePlannedMoveInputs(
    MovementState state,
    Point currentPos,
    LocalMap map
  ) {
    if (state == null) {
      logWarning("Null movement state");
      return false;
    }
    if (currentPos == null) {
      logWarning("Null current position");
      return false;
    }
    if (map == null) {
      logWarning("Null map");
      return false;
    }
    if (state.targetPosition == null) {
      logWarning("Null target position");
      return false;
    }
    return true;
  }

  private boolean isValidMove(
    Point currentPos,
    String direction,
    LocalMap map,
    int size,
    String blockDirection
  ) {
    try {
      if (direction == null) return false;

      List<String> availableDirections = getValidDirections(
        "validation",
        currentPos,
        map,
        size,
        blockDirection,
        null
      );

      return availableDirections.contains(direction);
    } catch (Exception e) {
      logger.warning(
        String.format("Error validating move: %s", e.getMessage())
      );
      return false;
    }
  }

  private boolean isTargetStillValid(MovementState state, LocalMap map) {
    try {
      if (state.targetType == null || state.targetPosition == null) {
        return false;
      }

      List<Point> targets = getTargetsOfType(map, state.targetType);
      return targets.contains(state.targetPosition);
    } catch (Exception e) {
      logger.warning("Error checking target validity: " + e.getMessage());
      return false;
    }
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

  private void logWarning(String message) {
    logger.warning(message);
  }

  private void logWarningf(String format, Object... args) {
    logger.warning(String.format(format, args));
  }
}
