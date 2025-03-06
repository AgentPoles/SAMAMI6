package jason.eis.movements;

import static jason.eis.movements.Exploration.RecomputeReason;

import jason.eis.LocalMap;
import jason.eis.LocalMap.ObstacleInfo;
import jason.eis.MI6Model;
import jason.eis.Point;
import jason.eis.movements.AgentCollisionHandler;
import jason.eis.movements.collision.CollisionResolution;
import java.util.*;
import java.util.concurrent.ConcurrentHashMap;
import java.util.logging.Logger;
import java.util.stream.Collectors;

public class RandomMovement implements MovementStrategy {
  private static final Logger logger = Logger.getLogger(
    RandomMovement.class.getName()
  );
  private static final int STUCK_THRESHOLD_STEPS = 3;
  private static final int HIGH_TRAFFIC_THRESHOLD = 2;
  private static final long PATH_TIMEOUT_MS = 10000; // 10 seconds

  private static final boolean DEBUG = true;
  private static final int MEMORY_SIZE = 5;
  private static final int ZONE_SIZE = 5;
  private static final double EXPLORATION_WEIGHT = 0.6;
  private static final double SAFETY_THRESHOLD = 3.0;
  private static final int MAX_ZONE_VISITS = 2;
  private static final double ZONE_WEIGHT = 0.5;

  // Use AgentCollisionHandler's constants instead
  private static final int AWARENESS_ZONE = 4;

  private final Random random = new Random();
  private final AgentCollisionHandler collisionHandler;
  private final Map<Point, ZoneInfo> zoneMemory = new ConcurrentHashMap<>();
  private final Map<String, Point> startingZones = new ConcurrentHashMap<>();

  // Constants
  private static final double ENTROPY_DECAY = 0.95;
  private static final double BASE_WEIGHT = 0.5;

  // Optimized constants
  private static final int HEAT_MAP_SIZE = 100; // Limit heat map size for memory efficiency

  // Dynamic weights
  private static final double BASE_ENTROPY_WEIGHT = 0.4;
  private static final double BASE_MOMENTUM_WEIGHT = 0.3;
  private static final double BASE_OBSTACLE_WEIGHT = 0.3;

  // Obstacle avoidance constants
  private static final double STATIC_OBSTACLE_DANGER = 1.0;
  private static final double DYNAMIC_OBSTACLE_DANGER = 1.5; // Agents are more dangerous
  private static final double DANGER_RADIUS = 3.0; // Influence radius of obstacles
  private static final double DANGER_DECAY = 0.7; // How quickly danger decreases with distance

  // Updated constants for obstacle avoidance
  private static final double IMMEDIATE_DANGER_THRESHOLD = 0.7;
  private static final double SAFE_SCORE_THRESHOLD = 0.3;
  private static final int PREDICTION_STEPS = 2;

  // Constants for static obstacle and boundary handling
  private static final double BOUNDARY_DANGER_WEIGHT = 1.5; // Boundaries are very dangerous
  private static final double STATIC_OBSTACLE_WEIGHT = 1.2;
  private static final int CORNER_AWARENESS = 4; // Extra awareness near corners

  // Zone-based exploration constants
  private static final int ZONE_MEMORY = 10;
  private static final double UNEXPLORED_BONUS = 0.8;

  // Add repulsion from starting zone
  private static final int STARTING_ZONE_REPULSION = 20; // Distance to maintain from start

  private final Map<String, Integer> stuckCounter = new ConcurrentHashMap<>();
  private final Map<String, Long> pathStartTimes = new ConcurrentHashMap<>();

  // Add ZoneInfo class
  private static class ZoneInfo {
    int visits = 0;
    long lastVisitTime = 0;
    double explorationScore = 1.0;

    void visit() {
      visits++;
      lastVisitTime = System.currentTimeMillis();
      explorationScore =
        Math.max(0.2, 1.0 - (visits / (double) MAX_ZONE_VISITS));
    }

    void decay() {
      // Gradually increase exploration score for zones not visited recently
      long timeSinceVisit = System.currentTimeMillis() - lastVisitTime;
      if (timeSinceVisit > 30000) { // 30 seconds
        explorationScore = Math.min(1.0, explorationScore + 0.1);
        visits = Math.max(0, visits - 1);
      }
    }
  }

  // Add these constants for improved collision handling
  private static final int COLLISION_MEMORY_SIZE = 5;
  private static final int STUCK_THRESHOLD = 3;
  private static final double BLOCK_CARRIER_BUFFER = 2.0;
  private static final int MAX_RECOVERY_ATTEMPTS = 3;

  // Add these constants
  private static final int MAX_BOUNDARY_ATTEMPTS = 5;
  private static final double BOUNDARY_ESCAPE_CHANCE = 0.8;

  private static final double AGENT_AVOIDANCE_WEIGHT = 0.4;
  private static final int AGENT_CRITICAL_DISTANCE = 1; // Direct adjacency
  private static final int AGENT_AWARENESS_DISTANCE = 3; // Awareness radius

  private final Exploration exploration;
  private final ObstacleManager obstacleManager = new ObstacleManager();
  private final BoundaryManager boundaryManager = new BoundaryManager();

  public RandomMovement(
    AgentCollisionHandler collisionHandler,
    Exploration exploration
  ) {
    this.collisionHandler = collisionHandler;
    this.exploration = exploration;
  }

  @Override
  public String getNextMove(String agName, LocalMap map) {
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
      if (map == null) {
        logger.warning(String.format("[%s] Map is null", agName));
        return getDefaultDirection();
      }

      Point currentPos = map.getCurrentPosition();
      if (currentPos == null) {
        logger.warning(String.format("[%s] Current position is null", agName));
        return getDefaultDirection();
      }

      // Get and validate available directions
      List<String> availableDirections = getAvailableDirections(
        map,
        currentPos
      );
      if (availableDirections.isEmpty()) {
        logger.warning(
          String.format(
            "[%s] No available directions at position %s",
            agName,
            currentPos
          )
        );
        return getDefaultDirection();
      }

      // Handle collision resolution
      try {
        CollisionResolution resolution = collisionHandler.resolveCollision(
          agName,
          currentPos,
          null,
          map,
          size,
          blockDirection,
          availableDirections
        );

        if (resolution != null) {
          handleCollisionResolution(agName, currentPos, resolution, map);
          return validateDirection(
            resolution.getDirection(),
            availableDirections
          );
        }
      } catch (Exception e) {
        logger.warning(
          String.format(
            "[%s] Error in collision handling: %s",
            agName,
            e.getMessage()
          )
        );
        // Continue with normal direction selection
      }

      // Choose direction based on exploration
      return chooseDirection(agName, currentPos, availableDirections, map);
    } catch (Exception e) {
      logger.severe(
        String.format(
          "[%s] Critical error in getNextDirection: %s",
          agName,
          e.getMessage()
        )
      );
      return getDefaultDirection();
    }
  }

  private List<String> getAvailableDirections(LocalMap map, Point currentPos) {
    try {
      List<String> directions = new ArrayList<>();
      String[] dirs = { "n", "s", "e", "w" };

      for (String dir : dirs) {
        Point nextPos = calculateNextPosition(currentPos, dir);
        if (isValidPosition(nextPos, map)) {
          directions.add(dir);
        }
      }

      return directions;
    } catch (Exception e) {
      logger.warning("Error getting available directions: " + e.getMessage());
      return Arrays.asList("n", "s", "e", "w"); // Return all directions as fallback
    }
  }

  private boolean isValidPosition(Point pos, LocalMap map) {
    try {
      return (
        pos != null &&
        map != null &&
        !map.hasObstacle(pos) &&
        !map.isOutOfBounds(pos)
      );
    } catch (Exception e) {
      logger.warning("Error checking position validity: " + e.getMessage());
      return false;
    }
  }

  private void handleCollisionResolution(
    String agName,
    Point currentPos,
    CollisionResolution resolution,
    LocalMap map
  ) {
    RecomputeReason reason;
    switch (resolution.getReason()) {
      case "STUCK":
        reason = RecomputeReason.STUCK;
        break;
      case "OSCILLATION":
        reason = RecomputeReason.OSCILLATION;
        break;
      default:
        return;
    }
    exploration.triggerPathRecompute(agName, currentPos, map, reason);
  }

  private String chooseDirection(
    String agName,
    Point currentPos,
    List<String> availableDirections,
    LocalMap map
  ) {
    try {
      String nextDirection = exploration.getNextDirection(
        agName,
        currentPos,
        availableDirections,
        map
      );
      return validateDirection(nextDirection, availableDirections);
    } catch (Exception e) {
      logger.warning(
        String.format(
          "[%s] Error choosing direction: %s",
          agName,
          e.getMessage()
        )
      );
      return getRandomDirection(availableDirections);
    }
  }

  private String validateDirection(
    String direction,
    List<String> availableDirections
  ) {
    if (
      direction == null ||
      direction.isEmpty() ||
      !availableDirections.contains(direction)
    ) {
      return availableDirections.isEmpty()
        ? getDefaultDirection()
        : availableDirections.get(random.nextInt(availableDirections.size()));
    }
    return direction;
  }

  private String getDefaultDirection() {
    try {
      return "n"; // Default fallback direction
    } catch (Exception e) {
      logger.severe(
        "Critical error getting default direction: " + e.getMessage()
      );
      return "n";
    }
  }

  private String getRandomDirection(List<String> availableDirections) {
    try {
      if (availableDirections == null || availableDirections.isEmpty()) {
        return getDefaultDirection();
      }
      return availableDirections.get(
        random.nextInt(availableDirections.size())
      );
    } catch (Exception e) {
      logger.warning("Error getting random direction: " + e.getMessage());
      return getDefaultDirection();
    }
  }

  private double evaluatePosition(Point targetPos, LocalMap map) {
    double score = 1.0;

    // Consider open spaces
    int openSpaces = countAdjacentOpenSpaces(targetPos, map);
    score *= (1.0 + (openSpaces / 8.0));

    // Consider distance to boundaries
    Map<String, Point> boundaries = map.getConfirmedBoundariesPositions();
    if (!boundaries.isEmpty()) {
      double minBoundaryDistance = Double.MAX_VALUE;
      for (Map.Entry<String, Point> entry : boundaries.entrySet()) {
        Point boundaryPoint = entry.getValue();
        double distance;

        switch (entry.getKey()) {
          case "n":
          case "s":
            distance = Math.abs(targetPos.y - boundaryPoint.y);
            break;
          case "e":
          case "w":
            distance = Math.abs(targetPos.x - boundaryPoint.x);
            break;
          default:
            continue;
        }
        minBoundaryDistance = Math.min(minBoundaryDistance, distance);
      }

      // Apply boundary distance penalty
      if (minBoundaryDistance < AWARENESS_ZONE) {
        score *= (minBoundaryDistance / AWARENESS_ZONE);
      }
    }

    return score;
  }

  private int countAdjacentOpenSpaces(Point pos, LocalMap map) {
    int count = 0;
    for (int dx = -1; dx <= 1; dx++) {
      for (int dy = -1; dy <= 1; dy++) {
        if (dx == 0 && dy == 0) continue;
        Point checkPos = new Point(pos.x + dx, pos.y + dy);
        if (!map.hasObstacle(checkPos) && !map.isOutOfBounds(checkPos)) {
          count++;
        }
      }
    }
    return count;
  }

  private String selectBestDirection(
    Map<String, Double> scores,
    List<String> availableDirections
  ) {
    if (scores.isEmpty()) {
      return getRandomDirection(availableDirections);
    }

    return scores
      .entrySet()
      .stream()
      .max(Map.Entry.comparingByValue())
      .map(Map.Entry::getKey)
      .orElseGet(() -> getRandomDirection(availableDirections));
  }

  private Point calculateTargetPosition(Point current, String direction) {
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

  private double getObstacleAvoidanceScore(
    String agName,
    LocalMap map,
    Point currentPos,
    Point targetPos
  ) {
    double score = 1.0;

    // Check if path is safe using LocalMap's improved path checking
    if (!map.isPathSafe(currentPos, targetPos)) {
      return 0.0; // Immediate danger
    }

    // Get dynamic obstacles and their predicted positions
    Map<Point, ObstacleInfo> dynamicObstacles = map.getDynamicObstacles();

    if (!dynamicObstacles.isEmpty()) {
      double minDanger = 1.0;

      for (ObstacleInfo obstacle : dynamicObstacles.values()) {
        // Check current and predicted positions
        for (int step = 0; step <= PREDICTION_STEPS; step++) {
          Point predictedPos = obstacle.predictPosition(step);
          double distance = euclideanDistance(targetPos, predictedPos);

          // Calculate danger based on distance and prediction step
          double stepDanger = calculateStepDanger(distance, step);
          minDanger = Math.min(minDanger, stepDanger);
        }
      }

      score *= minDanger;
    }

    return score;
  }

  private double calculateStepDanger(double distance, int predictionStep) {
    if (distance <= LocalMap.CRITICAL_DISTANCE) {
      return 0.0; // Maximum danger
    }

    if (distance >= LocalMap.AWARENESS_DISTANCE) {
      return 1.0; // No danger
    }

    // Danger decreases with distance and prediction uncertainty
    double baseDanger =
      (distance - LocalMap.CRITICAL_DISTANCE) /
      (LocalMap.AWARENESS_DISTANCE - LocalMap.CRITICAL_DISTANCE);

    // Reduce danger impact for further future predictions
    return baseDanger * (1.0 + predictionStep * 0.2);
  }

  private double euclideanDistance(Point p1, Point p2) {
    double dx = p1.x - p2.x;
    double dy = p1.y - p2.y;
    return Math.sqrt(dx * dx + dy * dy);
  }

  private int manhattanDistance(Point p1, Point p2) {
    return Math.abs(p1.x - p2.x) + Math.abs(p1.y - p2.y);
  }

  private String getBreakoutDirection(
    List<String> availableDirections,
    String lastDirection
  ) {
    if (lastDirection != null) {
      availableDirections.remove(lastDirection);
      availableDirections.remove(getOppositeDirection(lastDirection));
    }

    if (availableDirections.isEmpty()) {
      return lastDirection;
    }

    return availableDirections.get(random.nextInt(availableDirections.size()));
  }

  private boolean isMovingTowardsBoundary(
    Point current,
    Point target,
    String boundaryDir
  ) {
    switch (boundaryDir) {
      case "n":
        return target.y < current.y;
      case "s":
        return target.y > current.y;
      case "e":
        return target.x > current.x;
      case "w":
        return target.x < current.x;
      default:
        return false;
    }
  }

  private double getDistanceToBoundary(
    LocalMap map,
    Point pos,
    String direction
  ) {
    int distance = 0;
    int x = pos.x;
    int y = pos.y;

    while (distance < AWARENESS_ZONE) {
      switch (direction) {
        case "n":
          y--;
          break;
        case "s":
          y++;
          break;
        case "e":
          x++;
          break;
        case "w":
          x--;
          break;
      }

      Point checkPos = new Point(x, y);
      if (map.isOutOfBounds(checkPos)) {
        return distance;
      }
      distance++;
    }
    return AWARENESS_ZONE;
  }

  private Point getCurrentZone(Point position) {
    return new Point(
      (int) Math.floor(position.x / (double) ZONE_SIZE),
      (int) Math.floor(position.y / (double) ZONE_SIZE)
    );
  }

  private double getZoneScore(Point targetPos) {
    Point zone = getCurrentZone(targetPos);
    ZoneInfo info = zoneMemory.computeIfAbsent(zone, k -> new ZoneInfo());
    info.decay();
    return info.explorationScore;
  }

  private double calculateSafetyScore(Point targetPos, LocalMap map) {
    Map<Point, ObstacleInfo> dynamicObstacles = map.getDynamicObstacles();

    if (dynamicObstacles.isEmpty()) return 1.0;

    double minDistance = Double.MAX_VALUE;
    for (ObstacleInfo obstacle : dynamicObstacles.values()) {
      Point obstaclePos = obstacle.getPosition();
      double distance = euclideanDistance(targetPos, obstaclePos);
      minDistance = Math.min(minDistance, distance);
    }

    return Math.min(1.0, minDistance / SAFETY_THRESHOLD);
  }

  private String getParallelDirection(
    String boundaryDirection,
    List<String> availableDirections
  ) {
    if (boundaryDirection == null) return null;

    // Get directions parallel to the boundary
    List<String> parallelDirs = new ArrayList<>();
    switch (boundaryDirection) {
      case "n":
      case "s":
        if (availableDirections.contains("e")) parallelDirs.add("e");
        if (availableDirections.contains("w")) parallelDirs.add("w");
        break;
      case "e":
      case "w":
        if (availableDirections.contains("n")) parallelDirs.add("n");
        if (availableDirections.contains("s")) parallelDirs.add("s");
        break;
    }

    if (parallelDirs.isEmpty()) return null;
    return parallelDirs.get(random.nextInt(parallelDirs.size()));
  }

  private String detectBoundaryDirection(LocalMap map) {
    Map<String, Point> boundaries = map.getConfirmedBoundariesPositions();
    if (boundaries.isEmpty()) return null;

    Point currentPos = map.getCurrentPosition();
    String closestBoundary = null;
    int minDistance = Integer.MAX_VALUE;

    for (Map.Entry<String, Point> entry : boundaries.entrySet()) {
      Point boundaryPos = entry.getValue();
      int distance;

      switch (entry.getKey()) {
        case "n":
        case "s":
          distance = Math.abs(boundaryPos.y - currentPos.y);
          break;
        case "e":
        case "w":
          distance = Math.abs(boundaryPos.x - currentPos.x);
          break;
        default:
          continue;
      }

      if (distance < minDistance) {
        minDistance = distance;
        closestBoundary = entry.getKey();
      }
    }

    return closestBoundary;
  }

  private String getDirectionBetweenZones(Point from, Point to) {
    StringBuilder direction = new StringBuilder();
    if (to.y < from.y) direction.append("n");
    if (to.y > from.y) direction.append("s");
    if (to.x > from.x) direction.append("e");
    if (to.x < from.x) direction.append("w");
    return direction.toString();
  }

  private boolean isNearBlockCarrier(LocalMap map, Point pos) {
    return map
      .getDynamicObstacles()
      .values()
      .stream()
      .anyMatch(
        obs ->
          obs.hasBlock() &&
          euclideanDistance(pos, obs.getPosition()) <= BLOCK_CARRIER_BUFFER
      );
  }

  private String handleAgentCollision(
    Point currentPos,
    List<String> availableDirections,
    LocalMap map
  ) {
    Map<Point, ObstacleInfo> dynamicObstacles = map.getDynamicObstacles();

    Point closestAgent = null;
    double minDistance = Double.MAX_VALUE;

    for (ObstacleInfo obstacle : dynamicObstacles.values()) {
      if (obstacle.isDynamic()) {
        double distance = euclideanDistance(currentPos, obstacle.getPosition());
        if (distance < minDistance) {
          minDistance = distance;
          closestAgent = obstacle.getPosition();
        }
      }
    }

    if (closestAgent == null) return null;

    List<String> perpendicularDirs = getPerpendicularDirections(
      getDirectionBetweenPoints(currentPos, closestAgent)
    );

    perpendicularDirs.retainAll(availableDirections);

    if (!perpendicularDirs.isEmpty()) {
      return perpendicularDirs.get(random.nextInt(perpendicularDirs.size()));
    }

    return null;
  }

  private String getDirectionBetweenPoints(Point from, Point to) {
    int dx = to.x - from.x;
    int dy = to.y - from.y;

    if (Math.abs(dx) > Math.abs(dy)) {
      return dx > 0 ? "e" : "w";
    } else {
      return dy > 0 ? "s" : "n";
    }
  }

  private boolean isNearAgent(Point pos, LocalMap map) {
    for (ObstacleInfo obstacle : map.getDynamicObstacles().values()) {
      if (
        obstacle.isDynamic() &&
        euclideanDistance(pos, obstacle.getPosition()) <=
        AGENT_AWARENESS_DISTANCE
      ) {
        return true;
      }
    }
    return false;
  }

  private String handleCollision(
    String agName,
    Point currentPos,
    LocalMap map,
    boolean hasBlock
  ) {
    List<String> availableDirections = Arrays.asList("n", "s", "e", "w");
    CollisionResolution resolution = collisionHandler.resolveCollision(
      agName,
      currentPos,
      null,
      map,
      hasBlock ? 2 : 1,
      null,
      availableDirections
    );
    return resolution != null ? resolution.getDirection() : null;
  }

  public void recordVisit(String agName, Point position) {
    if (!pathStartTimes.containsKey(agName)) {
      pathStartTimes.put(agName, System.currentTimeMillis());
    }
  }

  // Move helper methods to main class
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

  private List<String> getParallelDirections(String boundaryDirection) {
    List<String> parallel = new ArrayList<>();
    if (boundaryDirection == null) return parallel;

    switch (boundaryDirection) {
      case "n":
      case "s":
        parallel.add("e");
        parallel.add("w");
        break;
      case "e":
      case "w":
        parallel.add("n");
        parallel.add("s");
        break;
    }
    return parallel;
  }

  private List<String> getPerpendicularDirections(String direction) {
    List<String> perpendicular = new ArrayList<>();
    if (direction == null) return perpendicular;

    switch (direction) {
      case "n":
      case "s":
        perpendicular.add("e");
        perpendicular.add("w");
        break;
      case "e":
      case "w":
        perpendicular.add("n");
        perpendicular.add("s");
        break;
    }
    return perpendicular;
  }

  private boolean isPathTimeout(String agName) {
    Long pathStart = pathStartTimes.get(agName);
    return (
      pathStart != null &&
      System.currentTimeMillis() - pathStart > PATH_TIMEOUT_MS
    );
  }
}
