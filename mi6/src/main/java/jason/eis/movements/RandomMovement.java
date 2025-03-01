package jason.eis.movements;

import jason.eis.LocalMap;
import jason.eis.LocalMap.ObstacleInfo;
import jason.eis.MI6Model;
import jason.eis.Point;
import jason.eis.movements.ObstacleMemory;
import java.util.*;
import java.util.concurrent.ConcurrentHashMap;
import java.util.logging.Logger;
import java.util.stream.Collectors;

public class RandomMovement implements MovementStrategy {
  private static final Logger logger = Logger.getLogger(
    RandomMovement.class.getName()
  );
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
  private final Map<String, DirectionMemory> directionMemory = new HashMap<>();
  private final Map<String, AgentZoneMemory> agentZoneMemories = new HashMap<>();
  private final AgentCollisionHandler collisionHandler = new AgentCollisionHandler();
  private final Map<String, EntropyMap> entropyMaps = new ConcurrentHashMap<>();
  private final Map<String, StaticObstacleInfo> staticObstacleMemory = new ConcurrentHashMap<>();
  private final Map<String, ObstacleMemory> obstacleMemories = new ConcurrentHashMap<>();
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

  private final Exploration exploration = new Exploration();

  private class DirectionMemory {
    private final Deque<String> lastMoves = new ArrayDeque<>(MEMORY_SIZE);
    private Point lastPosition;
    private int stuckCount = 0;
    private int recoveryAttempts = 0;
    private long lastStuckTime = 0;
    private Set<String> failedDirections = new HashSet<>();

    // Add weight fields
    public double obstacleWeight = BASE_OBSTACLE_WEIGHT;
    public double entropyWeight = BASE_ENTROPY_WEIGHT;
    public double momentumWeight = BASE_MOMENTUM_WEIGHT;

    // Add these fields
    private int boundaryAttempts = 0;
    private boolean isOnBoundary = false;
    private String lastBoundaryDirection = null;

    void addMove(String direction) {
      if (lastMoves.size() >= MEMORY_SIZE) {
        lastMoves.removeLast();
      }
      lastMoves.addFirst(direction);
    }

    String getLastDirection() {
      return lastMoves.isEmpty() ? null : lastMoves.getFirst();
    }

    void recordCollision(String direction) {
      failedDirections.add(direction);
      stuckCount++;
      lastStuckTime = System.currentTimeMillis();
    }

    void clearCollisionMemory() {
      failedDirections.clear();
      stuckCount = 0;
      recoveryAttempts = 0;
    }

    boolean shouldAttemptRecovery() {
      return isStuck() && recoveryAttempts < MAX_RECOVERY_ATTEMPTS;
    }

    String getRecoveryDirection(
      List<String> availableDirections,
      String lastDirection
    ) {
      recoveryAttempts++;

      // Special handling for boundary stuck situations
      if (isOnBoundary) {
        boundaryAttempts++;
        if (boundaryAttempts > MAX_BOUNDARY_ATTEMPTS) {
          // Force a move away from boundary
          String escapeDir = getOppositeDirection(lastBoundaryDirection);
          if (availableDirections.contains(escapeDir)) {
            boundaryAttempts = 0;
            isOnBoundary = false;
            return escapeDir;
          }
        }

        // Try to move along the boundary
        List<String> parallelDirs = getParallelDirections(
          lastBoundaryDirection
        );
        parallelDirs.retainAll(availableDirections);
        if (
          !parallelDirs.isEmpty() &&
          random.nextDouble() < BOUNDARY_ESCAPE_CHANCE
        ) {
          return parallelDirs.get(random.nextInt(parallelDirs.size()));
        }
      }

      // Existing recovery logic
      List<String> safeDirections = availableDirections
        .stream()
        .filter(d -> !failedDirections.contains(d))
        .collect(Collectors.toList());

      if (safeDirections.isEmpty()) {
        failedDirections.clear();
        return getOppositeDirection(lastDirection);
      }

      List<String> perpendicularDirs = getPerpendicularDirections(
        lastDirection
      );
      perpendicularDirs.retainAll(safeDirections);

      if (!perpendicularDirs.isEmpty()) {
        return perpendicularDirs.get(random.nextInt(perpendicularDirs.size()));
      }

      return safeDirections.get(random.nextInt(safeDirections.size()));
    }

    boolean isStuck() {
      return stuckCount >= 3;
    }
  }

  private static class StaticObstacleInfo {
    final Set<Point> nearbyObstacles = new HashSet<>();
    final Set<String> boundaryDirections = new HashSet<>();
    boolean isNearCorner = false;

    void clear() {
      nearbyObstacles.clear();
      boundaryDirections.clear();
      isNearCorner = false;
    }
  }

  private static class AgentZoneMemory {
    Deque<Point> visitedZones = new ArrayDeque<>(ZONE_MEMORY);
    Map<Point, Integer> zoneVisits = new HashMap<>();
    Point currentZone;
    String targetDirection = null;
    int stepsInCurrentZone = 0;
    long zoneEntryTime = System.currentTimeMillis();

    void recordZone(Point zone) {
      if (!zone.equals(currentZone)) {
        if (visitedZones.size() >= ZONE_MEMORY) {
          visitedZones.removeLast();
        }
        visitedZones.addFirst(zone);
        currentZone = zone;
        stepsInCurrentZone = 0;
        zoneEntryTime = System.currentTimeMillis();
      }
      zoneVisits.merge(zone, 1, Integer::sum);
      stepsInCurrentZone++;

      // Force direction change if stuck in zone too long
      if (System.currentTimeMillis() - zoneEntryTime > 5000) { // 5 seconds
        targetDirection = null;
      }
    }

    boolean isZoneOvervisited(Point zone) {
      return (
        stepsInCurrentZone > ZONE_SIZE ||
        zoneVisits.getOrDefault(zone, 0) >= MAX_ZONE_VISITS
      );
    }
  }

  private static class EntropyMap {
    private final Map<Point, Double> heatMap = new HashMap<>();
    private long lastCleanup = System.currentTimeMillis();

    void updateHeat(Point pos) {
      heatMap.merge(pos, 1.0, Double::sum);
      cleanup();
    }

    private void cleanup() {
      if (System.currentTimeMillis() - lastCleanup > 10000) {
        heatMap.entrySet().removeIf(entry -> entry.getValue() < 0.1);
        lastCleanup = System.currentTimeMillis();
      }
    }

    double getHeat(Point pos) {
      return heatMap.getOrDefault(pos, 0.0);
    }

    void decayHeat() {
      heatMap.replaceAll((k, v) -> v * ENTROPY_DECAY);
    }
  }

  // This is the method called by RequestGuidance
  public String getNextDirection(String agName, LocalMap map) {
    if (map == null) {
      logger.warning("Map is null for agent " + agName);
      return "e"; // Default direction
    }
    return getNextMove(agName, map);
  }

  @Override
  public String getNextMove(String agName, LocalMap map) {
    Point currentPos = map.getCurrentPosition();
    if (currentPos == null) return null;

    if (DEBUG) {
      logger.info(
        String.format(
          "[%s] CHECKING MOVES - Current position: %s, Known boundaries: %s",
          agName,
          currentPos,
          map.getConfirmedBoundariesPositions()
        )
      );
      map.logBoundaryState();
    }

    // Get available directions (filtering out static obstacles and boundaries)
    List<String> availableDirections = Arrays
      .asList("n", "s", "e", "w")
      .stream()
      .filter(
        dir -> {
          Point next = calculateNextPosition(currentPos, dir);
          boolean forbidden = map.isForbidden(next);

          if (DEBUG && forbidden) {
            logger.info(
              String.format(
                "[%s] FORBIDDEN MOVE - Direction %s to %s is forbidden (obstacle or boundary)",
                agName,
                dir,
                next
              )
            );
          }

          return !forbidden;
        }
      )
      .collect(Collectors.toList());

    if (DEBUG) {
      logger.info(
        String.format(
          "[%s] AVAILABLE MOVES - From %s can move: %s",
          agName,
          currentPos,
          availableDirections.isEmpty()
            ? "none"
            : String.join(", ", availableDirections)
        )
      );
    }

    // If no available moves, try to find a safe direction
    if (availableDirections.isEmpty()) {
      if (DEBUG) {
        logger.warning(
          String.format(
            "[%s] NO AVAILABLE MOVES at %s - Attempting recovery",
            agName,
            currentPos
          )
        );
      }
      return null; // Let the agent's decision-making handle the stuck situation
    }

    // Use our improved direction selection
    String chosenDirection = chooseDirection(
      currentPos,
      availableDirections,
      map
    );
    return chosenDirection;
  }

  // Add method to record failed moves (called from ASL)
  public void recordFailedMove(String agName, String attemptedDirection) {
    Point currentPos = MI6Model
      .getInstance()
      .getAgentMap(agName)
      .getCurrentPosition();
    collisionHandler.recordFailedMove(agName, currentPos, attemptedDirection);
  }

  private String chooseDirection(
    Point currentPos,
    List<String> availableDirections,
    LocalMap map
  ) {
    String agName = Thread.currentThread().getName();
    Map<String, Double> scores = new HashMap<>();

    for (String direction : availableDirections) {
      // Base random factor
      double randomFactor = 0.8 + random.nextDouble() * 0.4;

      // Exploration score
      double explorationScore = exploration.scoreDirection(
        agName,
        currentPos,
        direction,
        map
      );

      // Safety score (existing obstacle avoidance)
      Point nextPos = calculateNextPosition(currentPos, direction);
      double safetyScore = evaluatePosition(nextPos, map);

      // Combine scores
      double finalScore =
        (explorationScore * 0.4) + (safetyScore * 0.4) + (randomFactor * 0.2);

      scores.put(direction, finalScore);
    }

    // Choose best direction
    String chosen = scores
      .entrySet()
      .stream()
      .max(Map.Entry.comparingByValue())
      .map(Map.Entry::getKey)
      .orElseGet(() -> getRandomDirection(availableDirections));

    // Record the visit after choosing direction
    exploration.recordVisit(agName, currentPos);

    return chosen;
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

  private String getDefaultDirection() {
    return "n"; // Default fallback direction
  }

  private String getRandomDirection(List<String> availableDirections) {
    if (availableDirections == null || availableDirections.isEmpty()) {
      return getDefaultDirection();
    }
    return availableDirections.get(random.nextInt(availableDirections.size()));
  }

  private Map<String, Double> calculateDirectionScores(
    String agName,
    LocalMap map,
    Point currentPos,
    List<String> availableDirections,
    String explorationDir,
    DirectionMemory memory,
    EntropyMap entropyMap
  ) {
    Map<String, Double> scores = new HashMap<>();

    for (String direction : availableDirections) {
      try {
        Point nextPos = calculateNextPosition(currentPos, direction);
        if (nextPos == null) continue;

        double safetyScore = getObstacleAvoidanceScore(
          agName,
          map,
          currentPos,
          nextPos
        );
        double entropyScore = 1.0 - entropyMap.getHeat(nextPos);
        double momentumScore = getMomentumScore(direction, memory);
        double explorationScore = direction.equals(explorationDir) ? 0.4 : 0.0;

        // Combine scores with randomization
        double randomFactor = random.nextDouble() * 0.1;
        scores.put(
          direction,
          (safetyScore * 0.3) +
          (entropyScore * 0.15) +
          (momentumScore * 0.15) +
          explorationScore +
          randomFactor
        );
      } catch (Exception e) {
        logger.warning(
          String.format(
            "Error calculating score for direction %s: %s",
            direction,
            e.getMessage()
          )
        );
        scores.put(direction, 0.0); // Safe default score
      }
    }
    return scores;
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

  private void updateMovementMemory(
    String agName,
    Point currentPos,
    String chosenDirection,
    DirectionMemory memory,
    EntropyMap entropyMap
  ) {
    try {
      Point newPos = calculateNextPosition(currentPos, chosenDirection);
      if (newPos != null) {
        memory.addMove(chosenDirection);
        entropyMap.updateHeat(newPos);
        entropyMap.decayHeat();
      }
    } catch (Exception e) {
      logger.warning(
        String.format(
          "Error updating movement memory for agent %s: %s",
          agName,
          e.getMessage()
        )
      );
    }
  }

  private List<String> getAvailableDirections(LocalMap map, Point currentPos) {
    List<String> directions = new ArrayList<>();
    Point[] adjacentPoints = {
      new Point(currentPos.x, currentPos.y - 1), // N
      new Point(currentPos.x, currentPos.y + 1), // S
      new Point(currentPos.x + 1, currentPos.y), // E
      new Point(currentPos.x - 1, currentPos.y), // W
    };
    String[] dirs = { "n", "s", "e", "w" };

    for (int i = 0; i < adjacentPoints.length; i++) {
      if (
        !map.hasObstacle(adjacentPoints[i]) &&
        !map.isOutOfBounds(adjacentPoints[i])
      ) {
        directions.add(dirs[i]);
      }
    }

    return directions;
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

  private double scoreDirection(
    String direction,
    Point targetPos,
    DirectionMemory memory,
    EntropyMap entropyMap
  ) {
    double entropyScore = 1.0 - entropyMap.getHeat(targetPos);
    double momentumScore = direction.equals(memory.getLastDirection())
      ? 1.0
      : 0.0;

    return (entropyScore + momentumScore) * BASE_WEIGHT;
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

  private double getMomentumScore(String direction, DirectionMemory memory) {
    String lastDir = memory.getLastDirection();
    if (lastDir == null) return 0.5;

    // Encourage some direction changes to prevent straight-line movement
    if (direction.equals(lastDir) && memory.lastMoves.size() >= 3) {
      boolean allSameDirection = memory
        .lastMoves.stream()
        .allMatch(d -> d.equals(lastDir));
      if (allSameDirection) {
        return 0.1; // Discourage continuing in the same direction for too long
      }
    }

    return direction.equals(lastDir) ? 0.8 : 0.4;
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
      // Remove the last direction and its opposite
      availableDirections.remove(lastDirection);
      availableDirections.remove(getOppositeDirection(lastDirection));
    }

    if (availableDirections.isEmpty()) {
      // If no other directions available, use any available direction
      return lastDirection;
    }

    // Choose a random direction from remaining options
    return availableDirections.get(random.nextInt(availableDirections.size()));
  }

  private void updateStaticObstacleInfo(
    String agName,
    LocalMap map,
    Point currentPos
  ) {
    StaticObstacleInfo info = staticObstacleMemory.computeIfAbsent(
      agName,
      k -> new StaticObstacleInfo()
    );
    info.clear();

    // Check surrounding area for static obstacles and boundaries
    int boundaryCount = 0;
    for (int dx = -CORNER_AWARENESS; dx <= CORNER_AWARENESS; dx++) {
      for (int dy = -CORNER_AWARENESS; dy <= CORNER_AWARENESS; dy++) {
        Point checkPos = new Point(currentPos.x + dx, currentPos.y + dy);

        // Check for static obstacles
        if (map.hasObstacle(checkPos)) {
          info.nearbyObstacles.add(checkPos);
        }

        // Check for boundaries
        if (map.isOutOfBounds(checkPos)) {
          boundaryCount++;
          // Determine boundary direction
          if (dx == 0 && dy < 0) info.boundaryDirections.add("n");
          if (dx == 0 && dy > 0) info.boundaryDirections.add("s");
          if (dx > 0 && dy == 0) info.boundaryDirections.add("e");
          if (dx < 0 && dy == 0) info.boundaryDirections.add("w");
        }
      }
    }

    // Check if we're near a corner (multiple boundary directions)
    info.isNearCorner = info.boundaryDirections.size() >= 2;
  }

  private double getStaticSafetyScore(
    String agName,
    LocalMap map,
    Point currentPos,
    Point targetPos
  ) {
    StaticObstacleInfo info = staticObstacleMemory.get(agName);
    if (info == null) return 1.0;

    double score = 1.0;

    // Check for immediate obstacles or boundaries
    if (map.hasObstacle(targetPos) || map.isOutOfBounds(targetPos)) {
      return 0.0;
    }

    // Boundary avoidance
    for (String boundaryDir : info.boundaryDirections) {
      if (isMovingTowardsBoundary(currentPos, targetPos, boundaryDir)) {
        double distanceToBoundary = getDistanceToBoundary(
          map,
          currentPos,
          boundaryDir
        );
        if (distanceToBoundary < AWARENESS_ZONE) {
          score *= (distanceToBoundary / (double) AWARENESS_ZONE);
        }
      }
    }

    // Corner avoidance (more cautious near corners)
    if (info.isNearCorner) {
      score *= 0.5;
    }

    // Static obstacle avoidance
    for (Point obstacle : info.nearbyObstacles) {
      double distance = euclideanDistance(targetPos, obstacle);
      if (distance < LocalMap.CRITICAL_DISTANCE) {
        score *= (distance / LocalMap.CRITICAL_DISTANCE);
      }
    }

    return score;
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

  private double calculateExplorationScore(
    Point nextPos,
    String agName,
    LocalMap map
  ) {
    Point currentZone = getCurrentZone(nextPos);
    AgentZoneMemory memory = agentZoneMemories.computeIfAbsent(
      agName,
      k -> new AgentZoneMemory()
    );

    // Stronger penalty for revisiting recent zones
    if (memory.visitedZones.contains(currentZone)) {
      int recency = new ArrayList<>(memory.visitedZones).indexOf(currentZone);
      return 0.3 / (recency + 1);
    }

    // Bonus for unexplored zones
    if (!memory.zoneVisits.containsKey(currentZone)) {
      return 1.0;
    }

    // Penalty based on number of visits
    int visits = memory.zoneVisits.getOrDefault(currentZone, 0);
    return Math.max(0.2, 1.0 - (visits * 0.2));
  }

  private boolean isDiagonalMove(String direction) {
    return direction.length() > 1; // Assuming diagonal moves are represented as "ne", "nw", "se", "sw"
  }

  private String getDirectionBetweenZones(Point from, Point to) {
    StringBuilder direction = new StringBuilder();
    if (to.y < from.y) direction.append("n");
    if (to.y > from.y) direction.append("s");
    if (to.x > from.x) direction.append("e");
    if (to.x < from.x) direction.append("w");
    return direction.toString();
  }

  private String chooseDirection(
    String agName,
    LocalMap map,
    List<String> availableDirections
  ) {
    Point currentPos = map.getCurrentPosition();
    Point currentZone = getCurrentZone(currentPos);
    AgentZoneMemory memory = agentZoneMemories.get(agName);

    if (memory == null) {
      memory = new AgentZoneMemory();
      agentZoneMemories.put(agName, memory);
    }

    // Record current zone
    memory.recordZone(currentZone);

    // Calculate scores for each direction
    String bestDirection = null;
    double bestScore = Double.NEGATIVE_INFINITY;

    for (String direction : availableDirections) {
      Point nextPos = calculateNextPosition(currentPos, direction);

      double explorationScore = calculateExplorationScore(nextPos, agName, map);
      double safetyScore = calculateSafetyScore(nextPos, map);
      double entropyScore = calculateEntropyScore(
        direction,
        memory.visitedZones
      );

      // Combined score with weights
      double score =
        (explorationScore * EXPLORATION_WEIGHT) +
        (safetyScore * 0.3) +
        (entropyScore * 0.3);

      if (score > bestScore) {
        bestScore = score;
        bestDirection = direction;
      }
    }

    return bestDirection;
  }

  private double calculateEntropyScore(
    String direction,
    Deque<Point> visitedZones
  ) {
    if (visitedZones.isEmpty()) return 1.0;

    // Count direction changes in recent history
    String lastDirection = null;
    int changes = 0;
    int total = 0;

    Point lastZone = null;
    for (Point zone : visitedZones) {
      if (lastZone != null) {
        String moveDirection = getDirectionBetweenZones(lastZone, zone);
        if (lastDirection != null && !moveDirection.equals(lastDirection)) {
          changes++;
        }
        lastDirection = moveDirection;
        total++;
      }
      lastZone = zone;
    }

    // Encourage direction changes if movement has been too linear
    double changeRatio = total > 0 ? (double) changes / total : 0;
    if (
      changeRatio < 0.3 &&
      lastDirection != null &&
      direction.equals(lastDirection)
    ) {
      return 0.2; // Penalize continuing in the same direction
    }

    return 0.8;
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

  private String handleBlockCarrierCollision(
    DirectionMemory memory,
    List<String> availableDirections
  ) {
    // Prioritize moving away from block carriers
    String lastDir = memory.getLastDirection();
    if (lastDir != null) {
      String oppositeDir = getOppositeDirection(lastDir);
      if (availableDirections.contains(oppositeDir)) {
        return oppositeDir;
      }
    }
    return memory.getRecoveryDirection(availableDirections, lastDir);
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

  private String calculateBestDirection(
    String agName,
    LocalMap map,
    Point currentPos,
    DirectionMemory memory
  ) {
    List<String> availableDirections = getAvailableDirections(map, currentPos);
    if (availableDirections.isEmpty()) {
      return getDefaultDirection();
    }

    // Check for immediate agent collision
    if (isNearAgent(currentPos, map)) {
      String collisionResolution = handleAgentCollision(
        memory,
        availableDirections,
        map
      );
      if (collisionResolution != null) {
        return collisionResolution;
      }
    }

    // Regular direction scoring
    Map<String, Double> scores = new HashMap<>();
    for (String direction : availableDirections) {
      Point targetPos = calculateNextPosition(currentPos, direction);
      if (targetPos == null) continue;

      double score = evaluatePosition(targetPos, map);

      // Add momentum factor
      if (direction.equals(memory.getLastDirection())) {
        score *= 1.2;
      }

      // Add randomization factor
      score *= (0.9 + random.nextDouble() * 0.2);

      scores.put(direction, score);
    }

    return scores
      .entrySet()
      .stream()
      .max(Map.Entry.comparingByValue())
      .map(Map.Entry::getKey)
      .orElse(getDefaultDirection());
  }

  private boolean isMovePossible(
    LocalMap map,
    Point currentPos,
    String direction
  ) {
    if (direction == null) return false;
    Point nextPos = calculateNextPosition(currentPos, direction);
    return !map.hasObstacle(nextPos) && !map.isOutOfBounds(nextPos);
  }

  // Add this helper method
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

  private String selectSafeDirection(
    List<String> availableDirections,
    LocalMap map
  ) {
    // Filter out directions that would hit boundaries
    List<String> safeDirections = availableDirections
      .stream()
      .filter(dir -> !MovementUtils.wouldHitBoundary(map, dir))
      .collect(Collectors.toList());

    if (safeDirections.isEmpty()) {
      // All directions hit boundaries, fall back to original list
      return getRandomDirection(availableDirections);
    }

    return getRandomDirection(safeDirections);
  }

  private boolean isNearBoundary(Point pos, LocalMap map) {
    Map<String, Point> boundaries = map.getConfirmedBoundariesPositions();
    if (boundaries.isEmpty()) {
      return false;
    }

    for (Map.Entry<String, Point> entry : boundaries.entrySet()) {
      String direction = entry.getKey();
      Point boundary = entry.getValue();

      // Calculate distance to boundary
      int distance;
      switch (direction) {
        case "n":
          distance = Math.abs(pos.y - boundary.y);
          if (distance <= AWARENESS_ZONE) return true;
          break;
        case "s":
          distance = Math.abs(pos.y - boundary.y);
          if (distance <= AWARENESS_ZONE) return true;
          break;
        case "e":
          distance = Math.abs(pos.x - boundary.x);
          if (distance <= AWARENESS_ZONE) return true;
          break;
        case "w":
          distance = Math.abs(pos.x - boundary.x);
          if (distance <= AWARENESS_ZONE) return true;
          break;
      }
    }
    return false;
  }

  private boolean isParallelMovement(
    Point current,
    Point target,
    Point otherAgent
  ) {
    // Check if movement is perpendicular to the line between agents
    int dx = target.x - current.x;
    int dy = target.y - current.y;
    int ax = otherAgent.x - current.x;
    int ay = otherAgent.y - current.y;

    // Calculate dot product
    double dotProduct =
      (dx * ax + dy * ay) /
      (Math.sqrt(dx * dx + dy * dy) * Math.sqrt(ax * ax + ay * ay));

    // Consider movement parallel if angle is close to 90 degrees
    return Math.abs(dotProduct) < 0.3; // About 75 degrees
  }

  private String handleAgentCollision(
    DirectionMemory memory,
    List<String> availableDirections,
    LocalMap map
  ) {
    Point currentPos = map.getCurrentPosition();
    Map<Point, ObstacleInfo> dynamicObstacles = map.getDynamicObstacles();

    // Find closest agent
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

    // Get perpendicular directions to move away
    List<String> perpendicularDirs = getPerpendicularDirections(
      getDirectionBetweenPoints(currentPos, closestAgent)
    );

    // Filter available perpendicular directions
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
    return collisionHandler.resolveCollision(
      agName,
      currentPos,
      null,
      map,
      hasBlock
    );
  }
}
