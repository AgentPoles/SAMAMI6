package jason.eis.movements;

import jason.eis.LocalMap;
import jason.eis.Point;
import java.util.*;
import java.util.concurrent.ConcurrentHashMap;
import java.util.stream.Collectors;

public class RandomMovement {
  private final Map<String, LocalMap> agentMaps;
  private final Map<String, DirectionMemory> directionMemory;
  private final Map<String, EntropyMap> entropyMaps;
  private final Random random = new Random();

  // Optimized constants
  private static final int MEMORY_SIZE = 5;
  private static final int HEAT_MAP_SIZE = 100; // Limit heat map size for memory efficiency
  private static final double ENTROPY_DECAY = 0.95;

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
  private static final int BOUNDARY_AWARENESS = 3; // Start avoiding boundaries at this distance
  private static final int CORNER_AWARENESS = 4; // Extra awareness near corners

  // Zone-based exploration constants
  private static final int ZONE_SIZE = 10;
  private static final double ZONE_WEIGHT = 0.3;
  private static final int MAX_ZONE_VISITS = 3;

  private static class EntropyMap {
    private final Map<Point, Double> heatMap = new HashMap<>();
    private long lastCleanup = System.currentTimeMillis();

    void updateHeat(Point pos) {
      // Add heat to current position
      heatMap.merge(pos, 1.0, Double::sum);

      // Periodic cleanup of old entries
      if (
        heatMap.size() > HEAT_MAP_SIZE ||
        System.currentTimeMillis() - lastCleanup > 10000
      ) {
        cleanup();
      }
    }

    private void cleanup() {
      // Remove oldest/coldest entries
      heatMap.entrySet().removeIf(entry -> entry.getValue() < 0.1);
      lastCleanup = System.currentTimeMillis();
    }

    double getHeat(Point pos) {
      return heatMap.getOrDefault(pos, 0.0);
    }

    void decayHeat() {
      heatMap.replaceAll((k, v) -> v * ENTROPY_DECAY);
    }
  }

  private static class DirectionMemory {
    Deque<String> lastMoves = new ArrayDeque<>(MEMORY_SIZE);
    Point lastPosition;
    int stuckCount = 0;
    double entropyWeight = BASE_ENTROPY_WEIGHT;
    double momentumWeight = BASE_MOMENTUM_WEIGHT;
    double obstacleWeight = BASE_OBSTACLE_WEIGHT;

    void addMove(String direction, Point newPosition) {
      if (lastMoves.size() >= MEMORY_SIZE) {
        lastMoves.removeLast();
      }
      lastMoves.addFirst(direction);

      // Update stuck status and adjust weights
      if (lastPosition != null && lastPosition.equals(newPosition)) {
        stuckCount++;
        adjustWeights(true);
      } else {
        stuckCount = 0;
        adjustWeights(false);
      }
      lastPosition = newPosition;
    }

    void adjustWeights(boolean isStuck) {
      if (isStuck) {
        // Reduce momentum, increase entropy when stuck
        momentumWeight *= 0.8;
        entropyWeight *= 1.2;
      } else {
        // Gradually return to base weights
        momentumWeight = (momentumWeight + BASE_MOMENTUM_WEIGHT) / 2;
        entropyWeight = (entropyWeight + BASE_ENTROPY_WEIGHT) / 2;
      }

      // Normalize weights
      double total = entropyWeight + momentumWeight + obstacleWeight;
      entropyWeight /= total;
      momentumWeight /= total;
      obstacleWeight /= total;
    }

    boolean isStuck() {
      return stuckCount >= 3;
    }

    String getLastDirection() {
      return lastMoves.isEmpty() ? null : lastMoves.getFirst();
    }
  }

  private static class BoundaryState {
    boolean followingBoundary = false;
    int spiralDepth = 0;
    String boundaryDirection = null;
    int boundaryFollowSteps = 0;
  }

  private final Map<String, BoundaryState> boundaryStates = new ConcurrentHashMap<>();

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

  private final Map<String, StaticObstacleInfo> staticObstacleMemory = new ConcurrentHashMap<>();

  private final Map<String, ObstacleMemory> obstacleMemories = new ConcurrentHashMap<>();

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

  private final Map<Point, ZoneInfo> zoneMemory = new ConcurrentHashMap<>();

  public RandomMovement(Map<String, LocalMap> agentMaps) {
    this.agentMaps = agentMaps;
    this.directionMemory = new ConcurrentHashMap<>();
    this.entropyMaps = new ConcurrentHashMap<>();
  }

  public String getNextDirection(String agName) {
    LocalMap map = agentMaps.get(agName);
    if (map == null) return null;

    Point currentPos = map.getCurrentPosition();

    // Update current zone info
    Point currentZone = getCurrentZone(currentPos);
    ZoneInfo currentZoneInfo = zoneMemory.computeIfAbsent(
      currentZone,
      k -> new ZoneInfo()
    );
    currentZoneInfo.visit();

    // Update static obstacle information
    updateStaticObstacleInfo(agName, map, currentPos);

    DirectionMemory memory = directionMemory.computeIfAbsent(
      agName,
      k -> new DirectionMemory()
    );
    EntropyMap entropyMap = entropyMaps.computeIfAbsent(
      agName,
      k -> new EntropyMap()
    );

    // Update entropy
    entropyMap.updateHeat(currentPos);

    // Get available directions considering both static and dynamic obstacles
    List<String> availableDirections = getAvailableDirections(map, currentPos);
    if (availableDirections.isEmpty()) return null;

    // Calculate comprehensive safety scores
    Map<String, DirectionScore> directionScores = new HashMap<>();

    for (String direction : availableDirections) {
      Point targetPos = calculateTargetPosition(currentPos, direction);

      double dynamicSafetyScore = getObstacleAvoidanceScore(
        agName,
        map,
        currentPos,
        targetPos
      );
      double staticSafetyScore = getStaticSafetyScore(
        agName,
        map,
        currentPos,
        targetPos
      );
      double entropyScore = 1.0 - entropyMap.getHeat(targetPos);
      double momentumScore = getMomentumScore(direction, memory);
      double zoneScore = getZoneScore(targetPos);

      // Combine dynamic and static safety scores
      double combinedSafetyScore = Math.min(
        dynamicSafetyScore,
        staticSafetyScore
      );

      directionScores.put(
        direction,
        new DirectionScore(
          direction,
          combinedSafetyScore,
          entropyScore,
          momentumScore,
          zoneScore
        )
      );
    }

    // Filter and select best direction
    String bestDirection = selectBestDirection(directionScores, memory);

    // Update memories
    memory.addMove(bestDirection, currentPos);
    entropyMap.decayHeat();

    return bestDirection;
  }

  private List<String> getAvailableDirections(LocalMap map, Point currentPos) {
    List<String> available = new ArrayList<>();
    String[] directions = { "n", "s", "e", "w" };

    for (String dir : directions) {
      Point targetPos = calculateTargetPosition(currentPos, dir);
      if (!map.hasObstacle(targetPos) && !map.isOutOfBounds(targetPos)) {
        available.add(dir);
      }
    }
    return available;
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
    Map<Point, LocalMap.DynamicObstacle> dynamicObstacles = map.getDynamicObstacles();

    if (!dynamicObstacles.isEmpty()) {
      double minDanger = 1.0;

      for (LocalMap.DynamicObstacle obstacle : dynamicObstacles.values()) {
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
    return direction.equals(lastDir) ? 1.0 : 0.0;
  }

  private String selectBestDirection(
    Map<String, DirectionScore> scores,
    DirectionMemory memory
  ) {
    // First, filter out immediately dangerous directions
    List<DirectionScore> safeScores = scores
      .values()
      .stream()
      .filter(score -> score.safetyScore > IMMEDIATE_DANGER_THRESHOLD)
      .sorted(
        (a, b) ->
          Double.compare(b.getTotalScore(memory), a.getTotalScore(memory))
      )
      .collect(Collectors.toList());

    // If we have safe directions, choose the best one
    if (!safeScores.isEmpty()) {
      return safeScores.get(0).direction;
    }

    // If no safe directions, choose the least dangerous one
    return scores
      .entrySet()
      .stream()
      .max(
        (a, b) ->
          Double.compare(a.getValue().safetyScore, b.getValue().safetyScore)
      )
      .map(Map.Entry::getKey)
      .orElse(null);
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
    // When stuck, explicitly avoid the last direction
    availableDirections.remove(lastDirection);
    if (availableDirections.isEmpty()) {
      return lastDirection; // If no other choice, use last direction
    }
    return availableDirections.get(random.nextInt(availableDirections.size()));
  }

  private Point calculateTargetPosition(Point current, String direction) {
    switch (direction.toLowerCase()) {
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

  private String handleBoundaryFollowing(
    String agName,
    LocalMap map,
    List<String> availableDirections
  ) {
    BoundaryState state = boundaryStates.computeIfAbsent(
      agName,
      k -> new BoundaryState()
    );

    if (state.followingBoundary) {
      if (state.boundaryFollowSteps > 5) { // Start spiraling inward
        state.followingBoundary = false;
        state.spiralDepth++;
        return getSpiralDirection(state, availableDirections);
      }
      // Continue following boundary
      state.boundaryFollowSteps++;
      return getParallelDirection(state.boundaryDirection, availableDirections);
    }

    // Check if we hit a boundary
    if (availableDirections.size() <= 2) { // Likely at a boundary
      state.followingBoundary = true;
      state.boundaryDirection = detectBoundaryDirection(map);
      state.boundaryFollowSteps = 0;
      return getParallelDirection(state.boundaryDirection, availableDirections);
    }

    return null; // No boundary handling needed
  }

  private static class DirectionScore {
    final String direction;
    final double safetyScore;
    final double entropyScore;
    final double momentumScore;
    final double zoneScore;

    DirectionScore(
      String direction,
      double safetyScore,
      double entropyScore,
      double momentumScore,
      double zoneScore
    ) {
      this.direction = direction;
      this.safetyScore = safetyScore;
      this.entropyScore = entropyScore;
      this.momentumScore = momentumScore;
      this.zoneScore = zoneScore;
    }

    double getTotalScore(DirectionMemory memory) {
      return (
        (safetyScore * memory.obstacleWeight) +
        (entropyScore * memory.entropyWeight) +
        (momentumScore * memory.momentumWeight) +
        (zoneScore * ZONE_WEIGHT)
      );
    }
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
        if (distanceToBoundary < BOUNDARY_AWARENESS) {
          score *= (distanceToBoundary / BOUNDARY_AWARENESS);
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
    Point checkPos = new Point(pos.x, pos.y);

    while (distance < BOUNDARY_AWARENESS) {
      switch (direction) {
        case "n":
          checkPos.y--;
          break;
        case "s":
          checkPos.y++;
          break;
        case "e":
          checkPos.x++;
          break;
        case "w":
          checkPos.x--;
          break;
      }

      if (map.isOutOfBounds(checkPos)) {
        return distance;
      }
      distance++;
    }
    return BOUNDARY_AWARENESS;
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
}
