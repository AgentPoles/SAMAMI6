package jason.eis.movements;

import jason.eis.LocalMap;
import jason.eis.LocalMap.ObstacleInfo;
import jason.eis.Point;
import jason.eis.movements.ObstacleMemory;
import java.util.*;
import java.util.concurrent.ConcurrentHashMap;
import java.util.stream.Collectors;

public class RandomMovement {
  private final Map<String, LocalMap> agentMaps;
  private final Map<String, DirectionMemory> directionMemory = new ConcurrentHashMap<>();
  private final Map<String, EntropyMap> entropyMaps = new ConcurrentHashMap<>();
  private final Random random = new Random();

  // Constants
  private static final int MEMORY_SIZE = 5;
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
  private static final int BOUNDARY_AWARENESS = 3; // Start avoiding boundaries at this distance
  private static final int CORNER_AWARENESS = 4; // Extra awareness near corners

  // Zone-based exploration constants
  private static final int ZONE_SIZE = 5;
  private static final int ZONE_MEMORY = 10;
  private static final double EXPLORATION_WEIGHT = 0.6;
  private static final double UNEXPLORED_BONUS = 0.8;

  // Add this constant
  private static final double SAFETY_THRESHOLD = 3.0; // Distance threshold for safety calculations

  // Add these constants
  private static final int MAX_ZONE_VISITS = 2;
  private static final double ZONE_WEIGHT = 0.5;

  // Add repulsion from starting zone
  private static final int STARTING_ZONE_REPULSION = 20; // Distance to maintain from start
  private final Map<String, Point> startingZones = new ConcurrentHashMap<>();

  private final Map<String, BoundaryState> boundaryStates = new ConcurrentHashMap<>();

  private final Map<String, StaticObstacleInfo> staticObstacleMemory = new ConcurrentHashMap<>();

  private final Map<String, ObstacleMemory> obstacleMemories = new ConcurrentHashMap<>();

  private final Map<String, AgentZoneMemory> agentZoneMemories = new ConcurrentHashMap<>();

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

  // Add zoneMemory field
  private final Map<Point, ZoneInfo> zoneMemory = new ConcurrentHashMap<>();

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

  private static class DirectionMemory {
    private final Deque<String> lastMoves = new ArrayDeque<>(MEMORY_SIZE);
    private Point lastPosition;
    private int stuckCount = 0;

    // Add weight fields
    public double obstacleWeight = BASE_OBSTACLE_WEIGHT;
    public double entropyWeight = BASE_ENTROPY_WEIGHT;
    public double momentumWeight = BASE_MOMENTUM_WEIGHT;

    void addMove(String direction, Point newPosition) {
      if (lastMoves.size() >= MEMORY_SIZE) {
        lastMoves.removeLast();
      }
      lastMoves.addFirst(direction);

      if (lastPosition != null && lastPosition.equals(newPosition)) {
        stuckCount++;
      } else {
        stuckCount = 0;
      }
      lastPosition = newPosition;
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
    int spiralTurns = 0;
    int spiralSteps = 1;
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

  public RandomMovement(Map<String, LocalMap> agentMaps) {
    this.agentMaps = agentMaps;
  }

  public String getNextDirection(String agName) {
    LocalMap map = agentMaps.get(agName);
    if (map == null) return null;

    Point currentPos = map.getCurrentPosition();
    List<String> availableDirections = getAvailableDirections(map, currentPos);

    if (availableDirections.isEmpty()) {
      return null;
    }

    DirectionMemory memory = directionMemory.computeIfAbsent(
      agName,
      k -> new DirectionMemory()
    );
    EntropyMap entropyMap = entropyMaps.computeIfAbsent(
      agName,
      k -> new EntropyMap()
    );

    // If stuck, try to break pattern
    if (memory.isStuck()) {
      return getOppositeDirection(memory.getLastDirection());
    }

    // Score each available direction
    String bestDirection = null;
    double bestScore = Double.NEGATIVE_INFINITY;

    for (String direction : availableDirections) {
      Point targetPos = calculateTargetPosition(currentPos, direction);
      double score = scoreDirection(direction, targetPos, memory, entropyMap);

      if (score > bestScore) {
        bestScore = score;
        bestDirection = direction;
      }
    }

    if (bestDirection != null) {
      Point newPos = calculateTargetPosition(currentPos, bestDirection);
      memory.addMove(bestDirection, newPos);
      entropyMap.updateHeat(newPos);
      entropyMap.decayHeat();
    }

    return bestDirection != null
      ? bestDirection
      : availableDirections.get(random.nextInt(availableDirections.size()));
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
    int x = pos.x;
    int y = pos.y;

    while (distance < BOUNDARY_AWARENESS) {
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

  private String getSpiralDirection(
    BoundaryState state,
    List<String> availableDirections
  ) {
    if (state.spiralTurns >= 4) {
      state.spiralTurns = 0;
      state.spiralSteps++;
    }

    String direction = null;
    switch (state.spiralTurns) {
      case 0:
        direction = "n";
        break;
      case 1:
        direction = "e";
        break;
      case 2:
        direction = "s";
        break;
      case 3:
        direction = "w";
        break;
    }

    if (availableDirections.contains(direction)) {
      state.spiralTurns++;
      return direction;
    }

    // If preferred direction is not available, try the next one
    state.spiralTurns++;
    return getSpiralDirection(state, availableDirections);
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
    Map<String, Point> boundaries = map.getConfirmedBoundaries();
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
    Point nextZone = getCurrentZone(nextPos);
    AgentZoneMemory memory = agentZoneMemories.computeIfAbsent(
      agName,
      k -> new AgentZoneMemory()
    );

    // Record starting zone if not already recorded
    Point startingZone = startingZones.computeIfAbsent(
      agName,
      k -> getCurrentZone(map.getCurrentPosition())
    );

    // Strong penalty for staying near starting zone
    double startingZoneDistance = euclideanDistance(nextZone, startingZone);
    if (startingZoneDistance < STARTING_ZONE_REPULSION) {
      return -1.0 + (startingZoneDistance / STARTING_ZONE_REPULSION);
    }

    // Heavily penalize overvisited zones
    if (memory.isZoneOvervisited(nextZone)) {
      return -0.8;
    }

    // Higher bonus for unexplored zones
    if (!memory.zoneVisits.containsKey(nextZone)) {
      return UNEXPLORED_BONUS;
    }

    // Stronger penalties for recently visited zones
    double recencyPenalty = 0;
    int index = 0;
    for (Point zone : memory.visitedZones) {
      if (zone.equals(nextZone)) {
        recencyPenalty = 0.2 * (ZONE_MEMORY - index) / ZONE_MEMORY;
        break;
      }
      index++;
    }

    // Add diagonal movement bonus to encourage wider exploration
    String directionToZone = getDirectionBetweenZones(
      memory.currentZone,
      nextZone
    );
    boolean isDiagonal = isDiagonalMove(directionToZone);
    double diagonalBonus = isDiagonal ? 0.2 : 0;

    return 0.3 - recencyPenalty + diagonalBonus;
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
