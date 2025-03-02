package jason.eis.movements.collision.handlers;

import jason.eis.LocalMap;
import jason.eis.LocalMap.MovementRecord;
import jason.eis.LocalMap.ObstacleInfo;
import jason.eis.Point;
import java.util.*;

public class StuckHandler {
  private final Random random = new Random();
  private static final int AGENT_PROXIMITY_RANGE = 3;
  private static final double DIRECTION_SCORE_THRESHOLD = 0.7;

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
      return null;
    }

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

    // Penalize directions towards agents
    for (Map.Entry<Point, ObstacleInfo> entry : dynamicObstacles.entrySet()) {
      Point agentPos = entry.getKey();
      double distance = getDistance(nextPos, agentPos);

      // Heavy penalty for getting closer to agents
      if (distance < getDistance(currentPos, agentPos)) {
        score *= 0.5;
      }

      // Additional penalty based on proximity
      if (distance <= AGENT_PROXIMITY_RANGE) {
        score *= (distance / AGENT_PROXIMITY_RANGE);
      }
    }

    // Bonus for moving away from recent positions
    List<MovementRecord> history = map.getMovementHistory();
    if (!history.isEmpty()) {
      Point lastPos = history.get(0).position;
      if (getDistance(nextPos, lastPos) > 1) {
        score *= 1.2;
      }
    }

    // Penalty for previously tried directions
    if (map.getTriedDirections().contains(direction)) {
      score *= 0.8;
    }

    // Bonus for perpendicular movement to last direction
    String lastDir = map.getLastDirection();
    if (lastDir != null && isPerpendicularDirection(direction, lastDir)) {
      score *= 1.3;
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
}
