package jason.eis.movements;

import jason.eis.LocalMap;
import jason.eis.LocalMap.ObstacleInfo;
import jason.eis.Point;
import java.util.*;
import java.util.concurrent.ConcurrentHashMap;
import java.util.logging.Logger;

public class AgentUntangler {
  private static final Logger logger = Logger.getLogger(
    AgentUntangler.class.getName()
  );
  private static final boolean DEBUG = false;

  // Constants for quick decisions
  private static final int CRITICAL_DISTANCE = 1;
  private static final int EMERGENCY_DISTANCE = 1;
  private static final int SAFE_DISTANCE = 3;

  // Fast lookup for agent states
  private final Map<String, UntangleState> agentStates = new ConcurrentHashMap<>();

  private static class UntangleState {
    Point position;
    Point target;
    boolean hasBlock;
    boolean isEmergency;
    Set<String> recentInteractions;
    long lastInteractionTime;

    UntangleState() {
      recentInteractions = Collections.newSetFromMap(new ConcurrentHashMap<>());
      reset();
    }

    void reset() {
      position = null;
      target = null;
      hasBlock = false;
      isEmergency = false;
      recentInteractions.clear();
      lastInteractionTime = 0;
    }

    void markEmergency() {
      isEmergency = true;
      lastInteractionTime = System.currentTimeMillis();
    }
  }

  public String untangle(String agentId, LocalMap map) {
    try {
      Point currentPos = map.getCurrentPosition();
      if (currentPos == null) return null;

      // Quick check if untangling is needed
      if (!needsUntangling(agentId, currentPos, map)) {
        return null;
      }

      // Update agent state
      UntangleState state = agentStates.computeIfAbsent(
        agentId,
        k -> new UntangleState()
      );
      updateState(state, currentPos, map);

      // Emergency resolution for critical situations
      if (state.isEmergency || isEmergencySituation(currentPos, map)) {
        state.markEmergency();
        return resolveEmergency(state, map);
      }

      // Standard untangling
      return calculateUntangleMove(state, map);
    } catch (Exception e) {
      logger.warning("Error in untangling: " + e.getMessage());
      return null;
    }
  }

  private boolean needsUntangling(
    String agentId,
    Point currentPos,
    LocalMap map
  ) {
    Map<Point, ObstacleInfo> obstacles = map.getDynamicObstacles();

    // Check for nearby agents
    for (Map.Entry<Point, ObstacleInfo> entry : obstacles.entrySet()) {
      Point otherPos = entry.getKey();
      if (otherPos.equals(currentPos)) continue;

      int distance = getManhattanDistance(currentPos, otherPos);
      if (distance <= CRITICAL_DISTANCE) {
        return true;
      }
    }
    return false;
  }

  private boolean isEmergencySituation(Point currentPos, LocalMap map) {
    Map<Point, ObstacleInfo> obstacles = map.getDynamicObstacles();

    // Check for immediate collisions or blocked paths
    for (Point otherPos : obstacles.keySet()) {
      if (otherPos.equals(currentPos)) continue;

      if (getManhattanDistance(currentPos, otherPos) <= EMERGENCY_DISTANCE) {
        return true;
      }
    }
    return false;
  }

  private void updateState(
    UntangleState state,
    Point currentPos,
    LocalMap map
  ) {
    state.position = currentPos;
    // Check if there's a block attachment
    state.hasBlock = map.getBlockAttachment() != null;
  }

  private String resolveEmergency(UntangleState state, LocalMap map) {
    // Find the safest escape direction
    Map<String, Double> directionScores = new HashMap<>();

    for (String dir : Arrays.asList("n", "s", "e", "w")) {
      Point nextPos = calculateNextPosition(state.position, dir);
      if (!isValidMove(nextPos, map)) continue;

      double score = scoreEmergencyMove(nextPos, map);
      directionScores.put(dir, score);
    }

    return getBestDirection(directionScores);
  }

  private String calculateUntangleMove(UntangleState state, LocalMap map) {
    Map<String, Double> directionScores = new HashMap<>();

    for (String dir : Arrays.asList("n", "s", "e", "w")) {
      Point nextPos = calculateNextPosition(state.position, dir);
      if (!isValidMove(nextPos, map)) continue;

      double score = scoreUntangleMove(nextPos, state, map);
      directionScores.put(dir, score);
    }

    return getBestDirection(directionScores);
  }

  private double scoreEmergencyMove(Point nextPos, LocalMap map) {
    double score = 100.0; // Base score
    Map<Point, ObstacleInfo> obstacles = map.getDynamicObstacles();

    // Heavily penalize moves close to other agents
    for (Point otherPos : obstacles.keySet()) {
      if (otherPos.equals(nextPos)) {
        return Double.NEGATIVE_INFINITY;
      }

      int distance = getManhattanDistance(nextPos, otherPos);
      score -= (SAFE_DISTANCE - distance) * 20;
    }

    return score;
  }

  private double scoreUntangleMove(
    Point nextPos,
    UntangleState state,
    LocalMap map
  ) {
    double score = 0.0;
    Map<Point, ObstacleInfo> obstacles = map.getDynamicObstacles();

    // Distance from other agents
    double minAgentDistance = Double.MAX_VALUE;
    for (Point otherPos : obstacles.keySet()) {
      if (otherPos.equals(state.position)) continue;

      int distance = getManhattanDistance(nextPos, otherPos);
      minAgentDistance = Math.min(minAgentDistance, distance);
    }

    // Score components
    score += minAgentDistance * 10; // Prefer moves away from other agents

    if (state.target != null) {
      int targetDistance = getManhattanDistance(nextPos, state.target);
      score -= targetDistance * 2; // Still consider target direction
    }

    if (state.hasBlock) {
      score += 20; // Priority for agents with blocks
    }

    return score;
  }

  private String getBestDirection(Map<String, Double> directionScores) {
    return directionScores
      .entrySet()
      .stream()
      .max(Map.Entry.comparingByValue())
      .map(Map.Entry::getKey)
      .orElse(null);
  }

  private boolean isValidMove(Point pos, LocalMap map) {
    return !map.isForbidden(pos) && !map.hasObstacle(pos);
  }

  private int getManhattanDistance(Point p1, Point p2) {
    return Math.abs(p1.x - p2.x) + Math.abs(p1.y - p2.y);
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
