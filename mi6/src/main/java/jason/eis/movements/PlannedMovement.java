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
  private static final int PATH_TIMEOUT = 5000; // 5 seconds before recalculating path
  private static final int MAX_TARGETS_TO_CHECK = 5;

  private final Search search;
  private final Map<String, MovementState> agentStates;

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
    try {
      if (map == null || currentPos == null || targetType == null) {
        logger.warning(
          "Invalid parameters in findNearestTarget: " +
          String.format(
            "map=%s, currentPos=%s, targetType=%s",
            map != null ? "valid" : "null",
            currentPos,
            targetType
          )
        );
        return null;
      }

      List<Point> targets = getTargetsOfType(map, targetType);
      if (targets.isEmpty()) {
        if (DEBUG) logger.info("No targets found for type: " + targetType);
        return null;
      }

      try {
        // Sort targets by Manhattan distance
        targets.sort(
          (a, b) -> {
            try {
              int distA = getManhattanDistance(currentPos, a);
              int distB = getManhattanDistance(currentPos, b);
              return Integer.compare(distA, distB);
            } catch (Exception e) {
              logger.warning("Error comparing targets: " + e.getMessage());
              return 0;
            }
          }
        );

        List<Point> nearestTargets = targets.subList(
          0,
          Math.min(MAX_TARGETS_TO_CHECK, targets.size())
        );
        return findBestTarget(currentPos, nearestTargets, map, targetType);
      } catch (Exception e) {
        logger.warning("Error processing targets: " + e.getMessage());
        return null;
      }
    } catch (Exception e) {
      logger.severe("Critical error in findNearestTarget: " + e.getMessage());
      return null;
    }
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
    try {
      if (agName == null || map == null) {
        logger.warning("Invalid parameters in getNextMove");
        return null;
      }

      MovementState state = agentStates.computeIfAbsent(
        agName,
        k -> new MovementState(null, null)
      );

      if (state.targetPosition == null) {
        return null;
      }

      if (state.needsNewPath()) {
        try {
          Search.PathResult path = search.findPath(
            map.getCurrentPosition(),
            state.targetPosition,
            map,
            state.targetType
          );

          if (path != null && path.success) {
            state.plannedPath = new ArrayList<>(path.directions); // Create defensive copy
            state.pathCalculatedTime = System.currentTimeMillis();
            state.currentPathIndex = 0;
          } else {
            return null;
          }
        } catch (Exception e) {
          logger.warning("Error calculating new path: " + e.getMessage());
          return null;
        }
      }

      if (
        state.plannedPath == null ||
        state.plannedPath.isEmpty() ||
        state.currentPathIndex >= state.plannedPath.size()
      ) {
        return null;
      }

      String nextMove = state.plannedPath.get(state.currentPathIndex);
      state.currentPathIndex++;
      return nextMove;
    } catch (Exception e) {
      logger.severe("Critical error in getNextMove: " + e.getMessage());
      return null;
    }
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
}
