package jason.eis.movements.collision.handlers;

import jason.eis.LocalMap;
import jason.eis.LocalMap.MovementRecord;
import jason.eis.LocalMap.ObstacleInfo;
import jason.eis.Point;
import java.util.*;

public class StuckHandler {
  private final Random random = new Random();
  private static final int AGENT_PROXIMITY_RANGE = 1;
  private static final double DIRECTION_SCORE_THRESHOLD = 0.7;
  private static final int YIELD_THRESHOLD = 3;
  private static final long YIELD_DURATION = 1000;
  private long lastYieldTime = 0;
  private int stuckCount = 0;

  private double getDistance(Point p1, Point p2) {
    return Math.sqrt(Math.pow(p2.x - p1.x, 2) + Math.pow(p2.y - p1.y, 2));
  }

  // Helper method for direction opposites
  private boolean isOpposite(String dir1, String dir2) {
    if (dir1 == null || dir2 == null) return false;
    switch (dir1) {
      case "n":
        return dir2.equals("s");
      case "s":
        return dir2.equals("n");
      case "e":
        return dir2.equals("w");
      case "w":
        return dir2.equals("e");
      default:
        return false;
    }
  }

  // Helper method for alternating patterns
  private boolean isAlternatingPattern(List<String> moves) {
    if (moves.size() != 4) return false;
    return (
      moves.get(0).equals(moves.get(2)) &&
      moves.get(1).equals(moves.get(3)) &&
      !moves.get(0).equals(moves.get(1))
    );
  }

  public String resolveStuck(
    String agentName,
    LocalMap map,
    List<String> availableDirections
  ) {
    if (!map.isStuck()) {
      stuckCount = 0;
      return null;
    }

    stuckCount++;
    Point currentPos = map.getCurrentPosition();
    int agentSize = map.getAgentSize();
    String blockAttachment = map.getBlockAttachment();
    Set<String> triedDirections = map.getTriedDirections();

    // Check if there are nearby agents
    Map<Point, ObstacleInfo> dynamicObstacles = map.getDynamicObstacles();
    boolean hasNearbyAgents = dynamicObstacles
      .values()
      .stream()
      .anyMatch(
        obs ->
          getDistance(currentPos, obs.getPosition()) <= AGENT_PROXIMITY_RANGE
      );

    // Handle different scenarios
    if (hasNearbyAgents) {
      return resolveAgentStuck(
        map,
        availableDirections,
        currentPos,
        dynamicObstacles
      );
    } else if (agentSize > 1 || blockAttachment != null) {
      return resolveBlockStuck(map, availableDirections, blockAttachment);
    }

    return resolveSimpleStuck(map, availableDirections, triedDirections);
  }

  private String resolveBlockStuck(
    LocalMap map,
    List<String> availableDirections,
    String blockAttachment
  ) {
    List<String> safeDirections = getSafeDirectionsWithBlock(
      availableDirections,
      blockAttachment
    );

    if (!safeDirections.isEmpty()) {
      return safeDirections.get(random.nextInt(safeDirections.size()));
    }

    List<String> rotationSafeDirections = getRotationSafeDirections(
      map,
      availableDirections,
      map.getCurrentPosition()
    );

    if (!rotationSafeDirections.isEmpty()) {
      return rotationSafeDirections.get(
        random.nextInt(rotationSafeDirections.size())
      );
    }

    if (!availableDirections.isEmpty()) {
      return availableDirections.get(
        random.nextInt(availableDirections.size())
      );
    }

    return null;
  }

  private List<String> getSafeDirectionsWithBlock(
    List<String> availableDirections,
    String blockAttachment
  ) {
    List<String> safeDirections = new ArrayList<>();

    for (String dir : availableDirections) {
      if (
        isParallelToAttachment(dir, blockAttachment) ||
        dir.equals(blockAttachment)
      ) {
        safeDirections.add(dir);
      }
    }

    return safeDirections;
  }

  private List<String> getRotationSafeDirections(
    LocalMap map,
    List<String> availableDirections,
    Point position
  ) {
    List<String> safeDirections = new ArrayList<>();

    for (String dir : availableDirections) {
      if (hasRotationSpace(map, position, dir)) {
        safeDirections.add(dir);
      }
    }

    return safeDirections;
  }

  private boolean hasRotationSpace(
    LocalMap map,
    Point position,
    String direction
  ) {
    Point nextPos = calculateNextPosition(position, direction);

    for (int dx = -1; dx <= 1; dx++) {
      for (int dy = -1; dy <= 1; dy++) {
        Point checkPos = new Point(nextPos.x + dx, nextPos.y + dy);
        if (map.hasObstacle(checkPos) || map.isOutOfBounds(checkPos)) {
          return false;
        }
      }
    }

    return true;
  }

  private boolean isParallelToAttachment(String direction, String attachment) {
    if (attachment == null) return true;

    switch (attachment) {
      case "n":
      case "s":
        return direction.equals("n") || direction.equals("s");
      case "e":
      case "w":
        return direction.equals("e") || direction.equals("w");
      default:
        return false;
    }
  }

  private List<String> getPerpendicularDirections(
    String lastDir,
    List<String> availableDirections
  ) {
    if (lastDir == null) return new ArrayList<>();

    List<String> perpendicular = new ArrayList<>();

    switch (lastDir) {
      case "n":
      case "s":
        if (availableDirections.contains("e")) perpendicular.add("e");
        if (availableDirections.contains("w")) perpendicular.add("w");
        break;
      case "e":
      case "w":
        if (availableDirections.contains("n")) perpendicular.add("n");
        if (availableDirections.contains("s")) perpendicular.add("s");
        break;
    }

    return perpendicular;
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

  private String resolveAgentStuck(
    LocalMap map,
    List<String> availableDirections,
    Point currentPos,
    Map<Point, ObstacleInfo> dynamicObstacles
  ) {
    // Score each available direction based on multiple factors
    Map<String, Double> directionScores = new HashMap<>();

    for (String direction : availableDirections) {
      Point nextPos = calculateNextPosition(currentPos, direction);
      double score = calculateDirectionScore(
        map,
        currentPos,
        nextPos,
        direction,
        dynamicObstacles
      );
      directionScores.put(direction, score);
    }

    // Get the best direction that exceeds our threshold
    Optional<Map.Entry<String, Double>> bestDirection = directionScores
      .entrySet()
      .stream()
      .filter(e -> e.getValue() >= DIRECTION_SCORE_THRESHOLD)
      .max(Map.Entry.comparingByValue());

    if (bestDirection.isPresent()) {
      return bestDirection.get().getKey();
    }

    // If no good direction found, try perpendicular to last movement
    List<String> perpendicularDirs = getPerpendicularDirections(
      map.getLastDirection(),
      availableDirections
    );
    if (!perpendicularDirs.isEmpty()) {
      return perpendicularDirs.get(random.nextInt(perpendicularDirs.size()));
    }

    // Last resort: random available direction
    return !availableDirections.isEmpty()
      ? availableDirections.get(random.nextInt(availableDirections.size()))
      : null;
  }

  private String resolveSimpleStuck(
    LocalMap map,
    List<String> availableDirections,
    Set<String> triedDirections
  ) {
    String lastDir = map.getLastDirection();
    if (lastDir != null) {
      String oppositeDir = getOppositeDirection(lastDir);
      if (
        oppositeDir != null &&
        !triedDirections.contains(oppositeDir) &&
        availableDirections.contains(oppositeDir)
      ) {
        return oppositeDir;
      }
    }

    List<String> untriedDirs = new ArrayList<>(availableDirections);
    untriedDirs.removeAll(triedDirections);
    if (!untriedDirs.isEmpty()) {
      return untriedDirs.get(random.nextInt(untriedDirs.size()));
    }

    return !availableDirections.isEmpty()
      ? availableDirections.get(random.nextInt(availableDirections.size()))
      : null;
  }

  private double calculateDirectionScore(
    LocalMap map,
    Point currentPos,
    Point nextPos,
    String direction,
    Map<Point, ObstacleInfo> dynamicObstacles
  ) {
    double score = 1.0;

    // Count agents moving towards the same point
    int agentsTowardsSamePoint = countAgentsTowardsSamePoint(
      nextPos,
      dynamicObstacles
    );
    if (agentsTowardsSamePoint > 0) {
      score *= Math.pow(0.5, agentsTowardsSamePoint);
    }

    // Enhanced agent avoidance scoring
    for (Map.Entry<Point, ObstacleInfo> entry : dynamicObstacles.entrySet()) {
      Point agentPos = entry.getKey();
      double currentDistance = getDistance(currentPos, agentPos);
      double nextDistance = getDistance(nextPos, agentPos);

      // Stronger penalty for moving towards agents
      if (nextDistance < currentDistance) {
        score *= 0.4; // More aggressive penalty
      }

      // Progressive penalty based on proximity
      if (nextDistance <= AGENT_PROXIMITY_RANGE) {
        double proximityFactor = nextDistance / AGENT_PROXIMITY_RANGE;
        score *= (0.5 + (0.5 * proximityFactor)); // More nuanced scaling
      }

      // Consider agent's predicted movement
      Point predictedAgentPos = entry.getValue().predictPosition(1);
      if (predictedAgentPos.equals(nextPos)) {
        score *= 0.3; // Heavy penalty for potential collision
      }
    }

    // Enhanced history consideration
    List<MovementRecord> history = map.getMovementHistory();
    if (!history.isEmpty()) {
      // Avoid recent positions more strongly
      for (int i = 0; i < Math.min(3, history.size()); i++) {
        Point histPos = history.get(i).position;
        if (nextPos.equals(histPos)) {
          score *= 0.7; // Penalty for revisiting recent positions
        }
      }
    }

    // Consider yielding behavior
    if (shouldYield()) {
      score *= 0.3;
    }

    // Enhanced penalties for tried directions
    if (map.getTriedDirections().contains(direction)) {
      score *= 0.6; // Stronger penalty for tried directions
    }

    // Bonus for perpendicular movement
    String lastDir = map.getLastDirection();
    if (lastDir != null && isPerpendicularDirection(direction, lastDir)) {
      score *= 1.5; // Increased bonus for perpendicular movement
    }

    return score;
  }

  private boolean isPerpendicularDirection(String dir1, String dir2) {
    if (dir1 == null || dir2 == null) return false;

    return (
      (dir1.equals("n") || dir1.equals("s")) &&
      (dir2.equals("e") || dir2.equals("w")) ||
      (dir1.equals("e") || dir1.equals("w")) &&
      (dir2.equals("n") || dir2.equals("s"))
    );
  }

  private boolean shouldYield() {
    long currentTime = System.currentTimeMillis();
    if (currentTime - lastYieldTime < YIELD_DURATION) {
      return true;
    }
    if (stuckCount >= YIELD_THRESHOLD) {
      lastYieldTime = currentTime;
      stuckCount = 0;
      return true;
    }
    return false;
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
}
