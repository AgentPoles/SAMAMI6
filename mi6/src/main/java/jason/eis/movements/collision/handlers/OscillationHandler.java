package jason.eis.movements.collision.handlers;

import jason.eis.LocalMap;
import jason.eis.LocalMap.ObstacleInfo;
import jason.eis.Point;
import java.util.*;
import java.util.logging.Logger;
import java.util.stream.Collectors;

public class OscillationHandler {
  private static final Logger logger = Logger.getLogger(
    OscillationHandler.class.getName()
  );
  private static final boolean DEBUG = true;
  private static final int AGENT_PROXIMITY_RANGE = 1;
  private static final double DIRECTION_WEIGHT = 0.7;
  private static final double DISTANCE_WEIGHT = 0.3;
  private static final int YIELD_THRESHOLD = 4; // Number of oscillations before yielding
  private static final long YIELD_DURATION = 500; // Time to yield in milliseconds
  private final Random random = new Random();
  private long lastYieldTime = 0;
  private int oscillationCount = 0;

  public String resolveOscillation(
    LocalMap localMap,
    Point currentPos,
    String intendedDirection,
    List<String> availableDirections,
    String blockAttachment
  ) {
    try {
      if (localMap.isOscillating()) {
        oscillationCount++;
      } else {
        oscillationCount = 0;
      }

      if (DEBUG) {
        logger.info(
          String.format(
            "Checking oscillation at %s moving %s (count: %d)",
            currentPos,
            intendedDirection,
            oscillationCount
          )
        );
      }

      // Check if we're oscillating
      if (localMap.isOscillating()) {
        // Check for nearby agents
        Map<Point, ObstacleInfo> dynamicObstacles = localMap.getDynamicObstacles();
        boolean hasNearbyAgents = dynamicObstacles
          .values()
          .stream()
          .anyMatch(
            obs ->
              getDistance(currentPos, obs.getPosition()) <=
              AGENT_PROXIMITY_RANGE
          );

        if (hasNearbyAgents) {
          return resolveAgentOscillation(
            localMap,
            currentPos,
            availableDirections,
            dynamicObstacles,
            blockAttachment
          );
        } else {
          return resolveSimpleOscillation(
            localMap,
            availableDirections,
            blockAttachment
          );
        }
      }

      return null;
    } catch (Exception e) {
      logger.severe("Error in resolveOscillation: " + e.getMessage());
      e.printStackTrace();
      return null;
    }
  }

  private String resolveAgentOscillation(
    LocalMap localMap,
    Point currentPos,
    List<String> availableDirections,
    Map<Point, ObstacleInfo> dynamicObstacles,
    String blockAttachment
  ) {
    // Score directions based on agent positions and movement patterns
    Map<String, Double> directionScores = new HashMap<>();

    for (String direction : availableDirections) {
      Point nextPos = calculateNextPosition(currentPos, direction);
      double score = calculateDirectionScore(
        localMap,
        currentPos,
        nextPos,
        direction,
        dynamicObstacles,
        blockAttachment
      );
      directionScores.put(direction, score);
    }

    // Filter out oscillating directions unless they're significantly better
    Set<String> oscillatingDirs = localMap.getOscillatingDirections();
    List<Map.Entry<String, Double>> sortedDirections = directionScores
      .entrySet()
      .stream()
      .sorted(Map.Entry.<String, Double>comparingByValue().reversed())
      .collect(Collectors.toList());

    // Try to find a non-oscillating direction with a good score
    Optional<Map.Entry<String, Double>> bestNonOscillating = sortedDirections
      .stream()
      .filter(e -> !oscillatingDirs.contains(e.getKey()))
      .findFirst();

    if (bestNonOscillating.isPresent()) {
      return bestNonOscillating.get().getKey();
    }

    // If all directions are oscillating, pick the best scoring one
    return sortedDirections.get(0).getKey();
  }

  private double calculateDirectionScore(
    LocalMap localMap,
    Point currentPos,
    Point nextPos,
    String direction,
    Map<Point, ObstacleInfo> dynamicObstacles,
    String blockAttachment
  ) {
    double score = 1.0;

    // Calculate average direction of nearby agents
    Vector2D agentMovementVector = calculateAgentMovementVector(
      dynamicObstacles
    );

    // Count agents moving towards the same point
    int agentsTowardsSamePoint = countAgentsTowardsSamePoint(
      nextPos,
      dynamicObstacles
    );

    // Heavily penalize moving towards points that other agents are targeting
    if (agentsTowardsSamePoint > 0) {
      score *= Math.pow(0.5, agentsTowardsSamePoint);
    }

    // Direction component: prefer perpendicular to average agent movement
    if (agentMovementVector != null) {
      double directionAlignment = getPerpendicularAlignment(
        direction,
        agentMovementVector
      );
      score *= (1.0 + (directionAlignment * DIRECTION_WEIGHT));
    }

    // Distance component with improved multi-agent handling
    double totalDistanceScore = 0;
    int agentCount = 0;
    for (Map.Entry<Point, ObstacleInfo> entry : dynamicObstacles.entrySet()) {
      Point agentPos = entry.getKey();
      double currentDistance = getDistance(currentPos, agentPos);
      double nextDistance = getDistance(nextPos, agentPos);

      // Penalize moves that decrease distance to any agent
      if (nextDistance < currentDistance) {
        score *= 0.8;
      }

      // Calculate average distance score
      totalDistanceScore += Math.min(nextDistance / AGENT_PROXIMITY_RANGE, 1.0);
      agentCount++;
    }

    if (agentCount > 0) {
      double avgDistanceScore = totalDistanceScore / agentCount;
      score *= (1.0 + (avgDistanceScore * DISTANCE_WEIGHT));
    }

    // Consider yielding behavior
    if (shouldYield()) {
      score *= 0.3; // Significant reduction in score to encourage waiting
    }

    // Penalize oscillating directions more severely with multiple agents
    if (localMap.getOscillatingDirections().contains(direction)) {
      score *= Math.pow(0.8, Math.min(agentCount, 3));
    }

    // Block attachment constraints
    if (
      blockAttachment != null && !isValidBlockMove(direction, blockAttachment)
    ) {
      score *= 0.5;
    }

    return score;
  }

  private int countAgentsTowardsSamePoint(
    Point targetPoint,
    Map<Point, ObstacleInfo> dynamicObstacles
  ) {
    int count = 0;
    for (ObstacleInfo obstacle : dynamicObstacles.values()) {
      Point predictedPos = obstacle.predictPosition(1);
      if (predictedPos.equals(targetPoint)) {
        count++;
      }
    }
    return count;
  }

  private boolean shouldYield() {
    long currentTime = System.currentTimeMillis();

    // Check if we're still in yield period
    if (currentTime - lastYieldTime < YIELD_DURATION) {
      return true;
    }

    // Start new yield if oscillation count exceeds threshold
    if (oscillationCount >= YIELD_THRESHOLD) {
      lastYieldTime = currentTime;
      oscillationCount = 0;
      return true;
    }

    return false;
  }

  private Vector2D calculateAgentMovementVector(
    Map<Point, ObstacleInfo> dynamicObstacles
  ) {
    double totalX = 0, totalY = 0;
    int count = 0;

    // Instead of using velocity directly, we'll use the current position
    // and predict movement based on position relative to current agent
    for (Map.Entry<Point, ObstacleInfo> entry : dynamicObstacles.entrySet()) {
      Point agentPos = entry.getValue().getPosition();
      // If the agent is moving (has a predicted position different from current)
      Point predictedPos = entry.getValue().predictPosition(1);
      if (!agentPos.equals(predictedPos)) {
        totalX += predictedPos.x - agentPos.x;
        totalY += predictedPos.y - agentPos.y;
        count++;
      }
    }

    return count > 0 ? new Vector2D(totalX / count, totalY / count) : null;
  }

  private double getPerpendicularAlignment(String direction, Vector2D vector) {
    double dx = 0, dy = 0;
    switch (direction) {
      case "n":
        dy = -1;
        break;
      case "s":
        dy = 1;
        break;
      case "e":
        dx = 1;
        break;
      case "w":
        dx = -1;
        break;
    }

    // Calculate dot product with perpendicular vector
    double perpX = -vector.y;
    double perpY = vector.x;
    return Math.abs(dx * perpX + dy * perpY);
  }

  private String resolveSimpleOscillation(
    LocalMap localMap,
    List<String> availableDirections,
    String blockAttachment
  ) {
    // Existing simple oscillation resolution logic
    if (availableDirections.isEmpty()) return null;

    Set<String> oscillatingDirs = localMap.getOscillatingDirections();
    String lastDir = localMap.getLastDirection();

    if (lastDir != null) {
      List<String> perpDirs = getPerpendicularDirections(
          lastDir,
          blockAttachment
        )
        .stream()
        .filter(availableDirections::contains)
        .collect(Collectors.toList());

      if (!perpDirs.isEmpty()) {
        return perpDirs.get(random.nextInt(perpDirs.size()));
      }
    }

    List<String> validDirs = availableDirections
      .stream()
      .filter(dir -> !oscillatingDirs.contains(dir))
      .collect(Collectors.toList());

    if (!validDirs.isEmpty()) {
      return validDirs.get(random.nextInt(validDirs.size()));
    }

    return availableDirections.get(random.nextInt(availableDirections.size()));
  }

  // Helper class for vector calculations
  private static class Vector2D {
    final double x, y;

    Vector2D(double x, double y) {
      this.x = x;
      this.y = y;
    }
  }

  // Keep existing helper methods...
  private double getDistance(Point p1, Point p2) {
    return Math.sqrt(Math.pow(p2.x - p1.x, 2) + Math.pow(p2.y - p1.y, 2));
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

  private boolean isValidBlockMove(String direction, String blockAttachment) {
    if (blockAttachment == null) return true;
    return !direction.equals(getOppositeDirection(blockAttachment));
  }

  // Keep existing getPerpendicularDirections and getOppositeDirection methods...
  private List<String> getPerpendicularDirections(
    String direction,
    String blockAttachment
  ) {
    List<String> perpDirs = new ArrayList<>();
    switch (direction) {
      case "n":
      case "s":
        perpDirs.add("e");
        perpDirs.add("w");
        break;
      case "e":
      case "w":
        perpDirs.add("n");
        perpDirs.add("s");
        break;
    }

    // If block attached, filter out conflicting directions
    if (blockAttachment != null) {
      perpDirs.removeIf(
        dir ->
          dir.equals(blockAttachment) ||
          dir.equals(getOppositeDirection(blockAttachment))
      );
    }

    return perpDirs;
  }

  private String getOppositeDirection(String direction) {
    if (direction == null) return null;
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
}
