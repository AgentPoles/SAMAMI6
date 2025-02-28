package jason.eis.movements;

import jason.eis.LocalMap;
import jason.eis.MI6Model;
import jason.eis.Point;
import java.util.*;
import java.util.logging.Logger;

public class PlannedMovement implements MovementStrategy {
  private static final Logger logger = Logger.getLogger(
    PlannedMovement.class.getName()
  );
  private static final boolean DEBUG = true;
  private static final int MAX_SEARCH_RANGE = 50; // Configurable max range
  private final Search search;
  private final Map<String, MovementState> agentStates;
  private static final int PATH_TIMEOUT = 5000; // 5 seconds before recalculating path

  public PlannedMovement() {
    this.search = new Search();
    this.agentStates = new HashMap<>();
  }

  private static class MovementState {
    Point targetPosition;
    Search.TargetType targetType;
    List<String> plannedPath;
    long pathCalculatedTime;
    int currentPathIndex;

    MovementState(Point target, Search.TargetType type) {
      this.targetPosition = target;
      this.targetType = type;
      this.plannedPath = new ArrayList<>();
      this.pathCalculatedTime = 0;
      this.currentPathIndex = 0;
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
    List<Point> targets = getTargetsOfType(map, targetType);
    if (DEBUG) {
      logger.info(
        String.format(
          "Finding nearest target from %s. Found %d targets of type %s",
          currentPos,
          targets.size(),
          targetType
        )
      );
    }

    if (targets.isEmpty()) {
      if (DEBUG) logger.info("No targets found");
      return null;
    }

    // First, sort targets by Manhattan distance to reduce pathfinding calls
    targets.sort(
      (a, b) -> {
        int distA = Math.abs(a.x - currentPos.x) + Math.abs(a.y - currentPos.y);
        int distB = Math.abs(b.x - currentPos.x) + Math.abs(b.y - currentPos.y);
        return Integer.compare(distA, distB);
      }
    );

    // Only check paths for the closest N targets
    int maxTargetsToCheck = 5; // Adjust this value as needed
    List<Point> nearestTargets = targets.subList(
      0,
      Math.min(maxTargetsToCheck, targets.size())
    );

    Point bestTarget = null;
    int minDistance = Integer.MAX_VALUE;

    for (Point target : nearestTargets) {
      int manhattanDist =
        Math.abs(target.x - currentPos.x) + Math.abs(target.y - currentPos.y);
      if (manhattanDist >= minDistance || manhattanDist > MAX_SEARCH_RANGE) {
        continue;
      }

      Search.PathResult path = search.findPath(
        currentPos,
        target,
        map,
        targetType
      );
      if (path.success && path.points.size() < minDistance) {
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
    }

    return bestTarget;
  }

  private List<Point> getTargetsOfType(
    LocalMap map,
    Search.TargetType targetType
  ) {
    switch (targetType) {
      case DISPENSER:
        return map.getDispensers();
      case BLOCK:
        return map.getBlocks();
      case GOAL:
        return map.getGoals();
      default:
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

  @Override
  public String getNextMove(String agName, LocalMap map) {
    MovementState state = agentStates.computeIfAbsent(
      agName,
      k -> new MovementState(null, null)
    );

    if (state.targetPosition == null || state.needsNewPath()) {
      // Don't try to pathfind if we don't have a target
      if (state.targetPosition == null) {
        return null;
      }

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

    // Check bounds before accessing
    if (
      state.plannedPath.isEmpty() ||
      state.currentPathIndex >= state.plannedPath.size()
    ) {
      return null;
    }

    String nextMove = state.plannedPath.get(state.currentPathIndex);
    state.currentPathIndex++;
    return nextMove;
  }

  public String getNextMove(String agName) {
    LocalMap map = MI6Model.getInstance().getAgentMap(agName);
    return getNextMove(agName, map);
  }

  public void setTarget(String agName, Point target, Search.TargetType type) {
    MovementState state = new MovementState(target, type);
    agentStates.put(agName, state);
  }

  public void clearTarget(String agName) {
    agentStates.remove(agName);
  }

  public boolean hasTarget(String agName) {
    return (
      agentStates.containsKey(agName) &&
      agentStates.get(agName).targetPosition != null
    );
  }

  public void moveSucceeded(String agName) {
    MovementState state = agentStates.get(agName);
    if (state != null) {
      state.currentPathIndex++;
    }
  }

  public void moveFailed(String agName) {
    agentStates.remove(agName);
  }
}
