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
  private static final int CRITICAL_DISTANCE = 2;
  private static final int EMERGENCY_DISTANCE = 1;
  private static final int SAFE_DISTANCE = 2;
  private static final double OSCILLATION_TIME_THRESHOLD = 2000; // 2 seconds

  // Fast lookup for agent states
  private final Map<String, UntangleState> agentStates = new ConcurrentHashMap<>();

  private static final Random RANDOM = new Random();
  private static final int MIN_WAIT = 200;
  private static final int MAX_WAIT = 100;

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

      // Check if untangling is needed
      if (!needsUntangling(agentId, currentPos, map)) {
        return null;
      }

      // Add random wait when agents are too close
      try {
        int waitTime = MIN_WAIT + RANDOM.nextInt(MAX_WAIT - MIN_WAIT);
        Thread.sleep(waitTime);
        if (DEBUG) {
          logger.info(
            "Agent " +
            agentId +
            " waiting for " +
            waitTime +
            "ms to avoid collision"
          );
        }
      } catch (InterruptedException e) {
        Thread.currentThread().interrupt();
      }

      // After waiting, check if the situation resolved itself
      if (!needsUntangling(agentId, currentPos, map)) {
        return null;
      }

      // If we still need to untangle after waiting, return the current direction
      // This allows the agent to continue its planned path
      return map.getLastDirection();
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
    // Get all available directions
    List<String> availableDirections = getAvailableDirections(
      state.position,
      map
    );
    if (availableDirections.isEmpty()) return null;

    // Score each direction based on multiple factors
    Map<String, Double> directionScores = new HashMap<>();

    for (String direction : availableDirections) {
      Point nextPos = calculateNextPosition(state.position, direction);
      double score = 0.0;

      // Avoid recent directions that led to oscillation
      if (state.recentInteractions.contains(direction)) {
        continue;
      }

      // Prefer directions away from other agents
      score += getAgentDistanceScore(nextPos, map) * 2.0;

      // Prefer directions with more open space
      score += getOpenSpaceScore(nextPos, map) * 1.5;

      // Penalize directions that lead to corners or walls
      score -= getTrappedScore(nextPos, map);

      directionScores.put(direction, score);
    }

    // Select best direction
    return selectBestDirection(directionScores);
  }

  private double getAgentDistanceScore(Point pos, LocalMap map) {
    double minDistance = Double.MAX_VALUE;
    for (Point agentPos : map.getDynamicObstaclePositions()) {
      double distance = calculateDistance(pos, agentPos);
      minDistance = Math.min(minDistance, distance);
    }
    return Math.min(1.0, minDistance / SAFE_DISTANCE);
  }

  private double getOpenSpaceScore(Point pos, LocalMap map) {
    int openSpaces = 0;
    for (int dx = -1; dx <= 1; dx++) {
      for (int dy = -1; dy <= 1; dy++) {
        Point checkPos = new Point(pos.x + dx, pos.y + dy);
        if (!map.hasObstacle(checkPos)) {
          openSpaces++;
        }
      }
    }
    return openSpaces / 9.0;
  }

  private double getTrappedScore(Point pos, LocalMap map) {
    int blockedDirections = 0;
    for (String dir : Arrays.asList("n", "s", "e", "w")) {
      Point checkPos = calculateNextPosition(pos, dir);
      if (map.hasObstacle(checkPos)) {
        blockedDirections++;
      }
    }
    return blockedDirections / 4.0;
  }

  private String selectBestDirection(Map<String, Double> scores) {
    if (scores.isEmpty()) return null;
    return scores
      .entrySet()
      .stream()
      .max(Map.Entry.comparingByValue())
      .map(Map.Entry::getKey)
      .orElse(null);
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

  private List<String> getAvailableDirections(Point pos, LocalMap map) {
    List<String> availableDirections = new ArrayList<>();
    for (String dir : Arrays.asList("n", "s", "e", "w")) {
      Point nextPos = calculateNextPosition(pos, dir);
      if (isValidMove(nextPos, map)) {
        availableDirections.add(dir);
      }
    }
    return availableDirections;
  }

  private double calculateDistance(Point p1, Point p2) {
    int dx = p1.x - p2.x;
    int dy = p1.y - p2.y;
    return Math.sqrt(dx * dx + dy * dy);
  }
}
