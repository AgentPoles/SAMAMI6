package jason.eis.movements;

import jason.eis.LocalMap;
import jason.eis.Point;
import jason.eis.movements.AgentCollisionHandler;
import jason.eis.movements.collision.CollisionResolution;
import jason.eis.movements.collision.data.BaseCollisionState;
import jason.eis.movements.collision.data.MovementRecord;
import java.util.*;
import java.util.concurrent.ConcurrentHashMap;
import java.util.stream.Collectors;

public class Exploration {
  private static final int ZONE_SIZE = 5;
  private static final double UNEXPLORED_BONUS = 2.0;
  private static final double REVISIT_PENALTY = 0.3;
  private static final int DECAY_TIME = 30000; // 30 seconds
  private static final double ADJACENT_ZONE_WEIGHT = 0.3;

  // Heat map constants
  private static final double HEAT_DECAY_RATE = 0.92;
  private static final double INITIAL_HEAT = 1.0;
  private static final double MIN_HEAT = 0.1;
  private static final int HEAT_RADIUS = 3;

  // New exploration constants
  private static final double DISTANCE_BONUS_FACTOR = 0.15;
  private static final int MAX_LOCAL_VISITS = 3;
  private static final double LOCAL_AREA_RADIUS = 8;

  // Add these new constants at the top
  private static final int MAX_PATH_LENGTH = 15;
  private static final int MIN_PATH_LENGTH = 5;
  private static final double PATH_SCORE_THRESHOLD = 0.7;

  private final Map<Point, ZoneInfo> zoneMemory = new ConcurrentHashMap<>();
  private final Map<String, Set<Point>> agentVisitedZones = new ConcurrentHashMap<>();

  // Add heat map
  private final Map<Point, Double> heatMap = new ConcurrentHashMap<>();
  private long lastHeatDecay = System.currentTimeMillis();

  // Add these new fields
  private final Map<String, List<Point>> currentPaths = new ConcurrentHashMap<>();
  private final Search pathfinder = new Search();

  // Add these fields at the top of the class
  private final Map<String, PathResult> activePaths = new ConcurrentHashMap<>();
  private static final int PATH_TIMEOUT = 10000; // 10 seconds
  private final Map<String, Long> pathStartTimes = new ConcurrentHashMap<>();

  private final ExplorationSearch explorationSearch = new ExplorationSearch();

  // Add collision handling fields
  private final BaseCollisionState collisionState;
  private final Map<String, Integer> stuckCounter = new ConcurrentHashMap<>();
  private static final int STUCK_THRESHOLD_STEPS = 3;

  public Exploration(BaseCollisionState collisionState) {
    this.collisionState = collisionState;
  }

  private static class ZoneInfo {
    int visits = 0;
    long lastVisitTime = 0;
    double explorationScore = 1.0;
    Set<String> visitingAgents = new HashSet<>();

    void visit(String agName) {
      visits++;
      lastVisitTime = System.currentTimeMillis();
      visitingAgents.add(agName);
      updateScore();
    }

    private void updateScore() {
      // Sharper decrease for frequently visited zones
      explorationScore = Math.max(0.1, 1.0 - (visits * 0.25));
      // Additional penalty for multiple agents visiting same zone
      if (visitingAgents.size() > 1) {
        explorationScore *= 0.8;
      }
    }

    void decay() {
      long timeSinceVisit = System.currentTimeMillis() - lastVisitTime;
      if (timeSinceVisit > DECAY_TIME) {
        explorationScore = Math.min(1.0, explorationScore + 0.15);
        // Clear old visiting agents
        if (timeSinceVisit > DECAY_TIME * 2) {
          visitingAgents.clear();
        }
      }
    }
  }

  public enum RecomputeReason {
    PATH_BLOCKED, // Current path blocked by obstacle/boundary
    STUCK, // Agent unable to make progress
    HIGH_TRAFFIC, // Multiple agents in same zone
    EXPLORATION_TIMEOUT, // Path too old or too long
    OSCILLATION, // Oscillation detected
    BOUNDARY_CONSTRAINT,
    OBSTACLE_CONSTRAINT,
  }

  public static class PathResult {
    public final Point startPosition;
    public final List<String> directions;
    public final Point targetPosition;
    public final String targetType;
    public final boolean success;
    public final RecomputeReason reason;

    // Primary constructor
    public PathResult(
      Point start,
      List<String> dirs,
      Point target,
      String type
    ) {
      this.startPosition = start;
      this.directions = new ArrayList<>(dirs);
      this.targetPosition = target;
      this.targetType = type;
      this.success = true;
      this.reason = null;
    }

    // Constructor for failed paths
    public PathResult(boolean success) {
      this.startPosition = null;
      this.directions = new ArrayList<>();
      this.targetPosition = null;
      this.targetType = null;
      this.success = success;
      this.reason = null;
    }

    // Constructor with reason
    public PathResult(
      Point start,
      List<String> dirs,
      Point target,
      String type,
      RecomputeReason reason
    ) {
      this.startPosition = start;
      this.directions = new ArrayList<>(dirs);
      this.targetPosition = target;
      this.targetType = type;
      this.success = true;
      this.reason = reason;
    }
  }

  public double scoreDirection(
    String agName,
    Point currentPos,
    String direction,
    LocalMap map
  ) {
    // Check if we have a current path
    List<Point> path = currentPaths.get(agName);
    if (path != null && !path.isEmpty()) {
      Point nextInPath = path.get(0);
      Point nextPos = calculateNextPosition(currentPos, direction);
      if (nextPos.equals(nextInPath)) {
        path.remove(0);
        if (path.isEmpty()) {
          currentPaths.remove(agName);
        }
        return 2.0; // Prioritize following the path
      }
    }

    Point nextPos = calculateNextPosition(currentPos, direction);
    Point nextZone = getZone(nextPos);

    // Get current zone info
    ZoneInfo info = zoneMemory.computeIfAbsent(nextZone, k -> new ZoneInfo());
    info.decay();

    // Decay heat map periodically
    updateHeatMap();

    // Calculate base score
    double score = calculateBaseScore(agName, nextZone, info);

    // Add distance bonus from current position
    double distanceBonus = calculateDistanceBonus(agName, nextZone);

    // Get heat penalty
    double heatScore = getHeatScore(nextZone);

    // Add local area saturation penalty
    double localAreaPenalty = calculateLocalAreaPenalty(agName, nextZone);

    // Combine all factors
    return (
      (score + distanceBonus) * (1.0 - heatScore) * (1.0 - localAreaPenalty)
    );
  }

  private double calculateBaseScore(String agName, Point zone, ZoneInfo info) {
    Set<Point> agentZones = agentVisitedZones.computeIfAbsent(
      agName,
      k -> new HashSet<>()
    );

    if (!agentZones.contains(zone)) {
      return UNEXPLORED_BONUS;
    } else {
      return info.explorationScore * REVISIT_PENALTY;
    }
  }

  private void updateHeatMap() {
    long now = System.currentTimeMillis();
    if (now - lastHeatDecay > 1000) { // Decay every second
      heatMap.replaceAll((k, v) -> Math.max(MIN_HEAT, v * HEAT_DECAY_RATE));
      lastHeatDecay = now;
    }
  }

  public void recordVisit(String agName, Point position) {
    Point zone = getZone(position);
    ZoneInfo info = zoneMemory.computeIfAbsent(zone, k -> new ZoneInfo());
    info.visit(agName);

    agentVisitedZones.computeIfAbsent(agName, k -> new HashSet<>()).add(zone);

    // Update heat map
    updateHeatInRadius(zone);
  }

  private void updateHeatInRadius(Point center) {
    for (int dx = -HEAT_RADIUS; dx <= HEAT_RADIUS; dx++) {
      for (int dy = -HEAT_RADIUS; dy <= HEAT_RADIUS; dy++) {
        Point p = new Point(center.x + dx, center.y + dy);
        double distance = Math.sqrt(dx * dx + dy * dy);
        if (distance <= HEAT_RADIUS) {
          // Exponential heat decay with distance
          double heatValue = INITIAL_HEAT * Math.exp(-distance / HEAT_RADIUS);
          heatMap.merge(p, heatValue, (old, new_) -> Math.min(1.0, old + new_));
        }
      }
    }
  }

  private double getHeatScore(Point zone) {
    return heatMap.getOrDefault(zone, 0.0);
  }

  // Method to get heat map for visualization or debugging
  public Map<Point, Double> getHeatMap() {
    return new HashMap<>(heatMap);
  }

  private double getAdjacentZonesScore(Point zone, String agName) {
    double totalScore = 0;
    int count = 0;

    // Check all adjacent zones
    for (int dx = -1; dx <= 1; dx++) {
      for (int dy = -1; dy <= 1; dy++) {
        if (dx == 0 && dy == 0) continue;

        Point adjacentZone = new Point(zone.x + dx, zone.y + dy);
        ZoneInfo info = zoneMemory.get(adjacentZone);

        if (info == null) {
          // Unexplored adjacent zones are good
          totalScore += 1.0;
        } else {
          totalScore += info.explorationScore;
        }
        count++;
      }
    }

    return count > 0 ? totalScore / count : 0;
  }

  private Point getZone(Point position) {
    return new Point(
      (int) Math.floor(position.x / (double) ZONE_SIZE),
      (int) Math.floor(position.y / (double) ZONE_SIZE)
    );
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

  private double calculateDistanceBonus(String agName, Point targetZone) {
    Set<Point> visitedZones = agentVisitedZones.get(agName);
    if (visitedZones == null || visitedZones.isEmpty()) {
      return 0.0;
    }

    // Find average position of recently visited zones
    Point centroid = calculateCentroid(visitedZones);
    double distance = euclideanDistance(centroid, targetZone);

    // Bonus increases with distance from recent exploration center
    return Math.min(1.0, distance * DISTANCE_BONUS_FACTOR);
  }

  private Point calculateCentroid(Set<Point> points) {
    int sumX = 0, sumY = 0;
    for (Point p : points) {
      sumX += p.x;
      sumY += p.y;
    }
    return new Point(sumX / points.size(), sumY / points.size());
  }

  private double calculateLocalAreaPenalty(String agName, Point zone) {
    int localVisits = countLocalVisits(agName, zone);
    return Math.min(1.0, localVisits / (double) MAX_LOCAL_VISITS);
  }

  private int countLocalVisits(String agName, Point zone) {
    Set<Point> visitedZones = agentVisitedZones.get(agName);
    if (visitedZones == null) return 0;

    return (int) visitedZones
      .stream()
      .filter(p -> euclideanDistance(p, zone) <= LOCAL_AREA_RADIUS)
      .count();
  }

  private double euclideanDistance(Point p1, Point p2) {
    int dx = p1.x - p2.x;
    int dy = p1.y - p2.y;
    return Math.sqrt(dx * dx + dy * dy);
  }

  // Add this new method
  public PathResult findExplorationPath(
    String agName,
    Point current,
    LocalMap map,
    List<String> safeDirections
  ) {
    // Check existing path first
    PathResult existingPath = getActivePath(agName);
    if (existingPath != null && !existingPath.directions.isEmpty()) {
      String nextDir = existingPath.directions.get(0);
      if (safeDirections.contains(nextDir)) {
        List<String> remainingDirections = new ArrayList<>(
          existingPath.directions
        );
        remainingDirections.remove(0);
        Point nextPos = calculateNextPosition(current, nextDir);
        return new PathResult(current, remainingDirections, nextPos, null);
      }
    }

    // Create exploration search parameters
    ExplorationSearch.SearchParams params = new ExplorationSearch.SearchParams();
    params.maxDepth = 10;
    params.unexploredWeight = 0.7; // High weight for unexplored areas
    params.heatWeight = 0.2; // Medium weight to avoid recently visited areas
    params.distanceWeight = 0.1; // Small weight for distance consideration

    // Find a path that maximizes exploration potential
    PathResult newPath = explorationSearch.findExplorationPath(
      current,
      map,
      heatMap,
      params
    );

    if (newPath.success) {
      setActivePath(agName, newPath);
      return newPath;
    }

    // If no good path found, just pick the best immediate direction
    String bestDirection = findBestImmediateDirection(
      current,
      map,
      safeDirections
    );
    if (bestDirection != null) {
      Point nextPos = calculateNextPosition(current, bestDirection);
      return new PathResult(
        current,
        Collections.singletonList(bestDirection),
        nextPos,
        null
      );
    }

    return new PathResult(false);
  }

  private String findBestImmediateDirection(
    Point current,
    LocalMap map,
    List<String> safeDirections
  ) {
    String bestDir = null;
    double bestScore = Double.NEGATIVE_INFINITY;

    for (String dir : safeDirections) {
      Point nextPos = calculateNextPosition(current, dir);
      if (map.isOutOfBounds(nextPos) || map.hasObstacle(nextPos)) continue;

      double score = 0.0;
      // Bonus for unexplored areas
      if (!map.isExplored(nextPos)) {
        score += UNEXPLORED_BONUS;
      }
      // Penalty for recently visited areas
      score -= heatMap.getOrDefault(nextPos, 0.0) * HEATMAP_WEIGHT;
      // Bonus for being away from boundaries
      if (!map.isNearBoundary(nextPos)) {
        score += 0.5;
      }

      if (score > bestScore) {
        bestScore = score;
        bestDir = dir;
      }
    }

    return bestDir;
  }

  // Add this helper method to convert path points to directions
  private List<String> convertPathToDirections(List<Point> path) {
    List<String> directions = new ArrayList<>();
    for (int i = 0; i < path.size() - 1; i++) {
      Point current = path.get(i);
      Point next = path.get(i + 1);

      if (next.x > current.x) directions.add("e"); else if (
        next.x < current.x
      ) directions.add("w"); else if (next.y > current.y) directions.add(
        "s"
      ); else if (next.y < current.y) directions.add("n");
    }
    return directions;
  }

  // Add this method to find least explored target
  private Point findLeastExploredTarget(Point current, LocalMap map) {
    Point bestTarget = null;
    double bestScore = -1;

    // Search in a radius around current position
    for (int dx = -MAX_PATH_LENGTH; dx <= MAX_PATH_LENGTH; dx++) {
      for (int dy = -MAX_PATH_LENGTH; dy <= MAX_PATH_LENGTH; dy++) {
        Point target = new Point(current.x + dx, current.y + dy);

        // Skip if out of bounds or obstacle
        if (map.isOutOfBounds(target) || map.hasObstacle(target)) {
          continue;
        }

        // Calculate exploration score
        double heatScore = getHeatScore(getZone(target));
        double distanceScore =
          1.0 - (Math.sqrt(dx * dx + dy * dy) / MAX_PATH_LENGTH);
        double score = (heatScore * 0.7) + (distanceScore * 0.3);

        if (score > bestScore) {
          bestScore = score;
          bestTarget = target;
        }
      }
    }

    return bestTarget;
  }

  private List<String> getAvailableDirections(Point current, LocalMap map) {
    // Implement our own getAvailableDirections since it's private in AgentCollisionHandler
    List<String> available = new ArrayList<>();
    for (String dir : Arrays.asList("n", "s", "e", "w")) {
      Point nextPos = calculateNextPosition(current, dir);
      if (!map.hasObstacle(nextPos) && !map.isOutOfBounds(nextPos)) {
        available.add(dir);
      }
    }
    return available;
  }

  private static boolean hasNearbyAgents(Point pos, LocalMap map) {
    for (Map.Entry<Point, LocalMap.ObstacleInfo> entry : map
      .getDynamicObstacles()
      .entrySet()) {
      Point agentPos = entry.getKey();
      double distance = Math.sqrt(
        Math.pow(pos.x - agentPos.x, 2) + Math.pow(pos.y - agentPos.y, 2)
      );
      if (distance <= 3.0) {
        return true;
      }
    }
    return false;
  }

  private String handleOscillation(
    String agentId,
    Point current,
    LocalMap map
  ) {
    List<String> availableDirections = getAvailableDirections(current, map);
    // Just return a random available direction that's not in the recent history
    List<String> recentDirs = getRecentDirections(agentId);
    availableDirections.removeAll(recentDirs);
    return availableDirections.isEmpty() ? null : availableDirections.get(0);
  }

  private String handleTrafficAvoidance(
    String agentId,
    Point current,
    LocalMap map
  ) {
    List<String> availableDirections = getAvailableDirections(current, map);
    // Choose direction with least traffic
    return findLeastTrafficDirection(current, availableDirections, map);
  }

  // Helper method to get recent directions
  private List<String> getRecentDirections(String agentId) {
    List<MovementRecord> history = collisionState.getMovementHistory(agentId);
    List<String> directions = new ArrayList<>();
    for (int i = 0; i < Math.min(3, history.size()); i++) {
      String dir = history.get(i).getIntendedDirection();
      if (dir != null) {
        directions.add(dir);
        directions.add(getOppositeDirection(dir));
      }
    }
    return directions;
  }

  // Helper method to find direction with least traffic
  private String findLeastTrafficDirection(
    Point current,
    List<String> directions,
    LocalMap map
  ) {
    String bestDir = null;
    int minAgents = Integer.MAX_VALUE;

    for (String dir : directions) {
      Point nextPos = calculateNextPosition(current, dir);
      int nearbyAgents = countNearbyAgents(nextPos, map);
      if (nearbyAgents < minAgents) {
        minAgents = nearbyAgents;
        bestDir = dir;
      }
    }

    return bestDir;
  }

  private int countNearbyAgents(Point pos, LocalMap map) {
    int count = 0;
    for (Map.Entry<Point, LocalMap.ObstacleInfo> entry : map
      .getDynamicObstacles()
      .entrySet()) {
      Point agentPos = entry.getKey();
      if (euclideanDistance(pos, agentPos) <= 3.0) {
        count++;
      }
    }
    return count;
  }

  private double calculateCollisionAvoidanceScore(Point pos, LocalMap map) {
    double score = 1.0;
    if (hasNearbyAgents(pos, map)) {
      score *= 0.5;
    }
    return score;
  }

  private static class ExplorationSearch {
    private static final String[] DIRECTIONS = { "n", "e", "s", "w" };
    private static final Map<String, Point> DIRECTION_VECTORS = new HashMap<>();
    private static final int MAX_DEPTH = 10;
    private static final double UNEXPLORED_WEIGHT = 0.6;
    private static final double HEAT_WEIGHT = 0.3;
    private static final double DISTANCE_WEIGHT = 0.1;

    static {
      DIRECTION_VECTORS.put("n", new Point(0, -1));
      DIRECTION_VECTORS.put("e", new Point(1, 0));
      DIRECTION_VECTORS.put("s", new Point(0, 1));
      DIRECTION_VECTORS.put("w", new Point(-1, 0));
    }

    public static class SearchParams {
      public int maxDepth = MAX_DEPTH;
      public double unexploredWeight = UNEXPLORED_WEIGHT;
      public double heatWeight = HEAT_WEIGHT;
      public double distanceWeight = DISTANCE_WEIGHT;

      public SearchParams() {} // Default constructor
    }

    private static class ExplorationNode
      implements Comparable<ExplorationNode> {
      Point position;
      ExplorationNode parent;
      String direction;
      double score;
      int depth;

      ExplorationNode(
        Point pos,
        ExplorationNode parent,
        String dir,
        double score,
        int depth
      ) {
        this.position = pos;
        this.parent = parent;
        this.direction = dir;
        this.score = score;
        this.depth = depth;
      }

      @Override
      public int compareTo(ExplorationNode other) {
        return Double.compare(other.score, this.score); // Higher score first
      }
    }

    public PathResult findExplorationPath(
      Point start,
      LocalMap map,
      Map<Point, Double> heatMap,
      SearchParams params
    ) {
      PriorityQueue<ExplorationNode> frontier = new PriorityQueue<>();
      Set<Point> visited = new HashSet<>();
      ExplorationNode bestNode = null;
      double bestScore = Double.NEGATIVE_INFINITY;

      frontier.add(new ExplorationNode(start, null, null, 0, 0));

      while (!frontier.isEmpty()) {
        ExplorationNode current = frontier.poll();

        if (visited.contains(current.position)) continue;
        visited.add(current.position);

        // Update best node if this has better exploration potential
        double currentScore = evaluateExplorationScore(current, map, heatMap);
        if (currentScore > bestScore) {
          bestScore = currentScore;
          bestNode = current;
        }

        // Stop if we've gone too deep
        if (current.depth >= params.maxDepth) continue;

        // Expand in all directions
        for (String dir : DIRECTIONS) {
          Point nextPos = getNextPosition(current.position, dir);
          if (isValidMove(nextPos, map) && !visited.contains(nextPos)) {
            double nextScore = calculateDirectionScore(
              nextPos,
              map,
              heatMap,
              current.depth + 1,
              params
            );
            frontier.add(
              new ExplorationNode(
                nextPos,
                current,
                dir,
                nextScore,
                current.depth + 1
              )
            );
          }
        }
      }

      if (bestNode == null || bestNode.parent == null) {
        return new PathResult(false);
      }

      return reconstructPath(bestNode);
    }

    private double calculateDirectionScore(
      Point pos,
      LocalMap map,
      Map<Point, Double> heatMap,
      int depth,
      SearchParams params
    ) {
      // Count unexplored neighbors
      int unexploredCount = countUnexploredNeighbors(pos, map);
      double unexploredScore = unexploredCount / 4.0; // Normalize to 0-1

      // Heat map penalty (lower is better)
      double heatPenalty = heatMap.getOrDefault(pos, 0.0);

      // Distance penalty
      double distancePenalty = depth / (double) MAX_DEPTH;

      // Collision avoidance score (check nearby agents)
      double collisionScore = calculateCollisionAvoidanceScore(pos, map);

      return (
        (unexploredScore * params.unexploredWeight) -
        (heatPenalty * params.heatWeight) -
        (distancePenalty * params.distanceWeight) +
        (collisionScore * 0.3)
      ); // Add collision avoidance weight
    }

    private double calculateCollisionAvoidanceScore(Point pos, LocalMap map) {
      double score = 1.0;
      if (hasNearbyAgents(pos, map)) {
        score *= 0.5;
      }
      return score;
    }

    private int countUnexploredNeighbors(Point pos, LocalMap map) {
      int count = 0;
      for (String dir : DIRECTIONS) {
        Point neighbor = getNextPosition(pos, dir);
        if (
          !map.isOutOfBounds(neighbor) &&
          !map.hasObstacle(neighbor) &&
          !map.isForbidden(neighbor)
        ) {
          count++;
        }
      }
      return count;
    }

    private double evaluateExplorationScore(
      ExplorationNode node,
      LocalMap map,
      Map<Point, Double> heatMap
    ) {
      // Similar to calculateDirectionScore but might include additional factors
      // like distance from starting point, total unexplored area in vicinity, etc.
      return calculateDirectionScore(
        node.position,
        map,
        heatMap,
        node.depth,
        new SearchParams()
      );
    }

    private boolean isValidMove(Point pos, LocalMap map) {
      return !map.hasObstacle(pos) && !map.isOutOfBounds(pos);
    }

    private Point getNextPosition(Point current, String direction) {
      Point vector = DIRECTION_VECTORS.get(direction);
      return new Point(current.x + vector.x, current.y + vector.y);
    }

    private PathResult reconstructPath(ExplorationNode node) {
      List<String> directions = new ArrayList<>();
      Point startPos = null;

      // Work backwards from target to start
      ExplorationNode current = node;
      while (current.parent != null) {
        directions.add(0, current.direction);
        if (startPos == null) {
          startPos = current.parent.position;
        }
        current = current.parent;
      }

      return new PathResult(startPos, directions, node.position, null);
    }

    // Add overloaded method for backward compatibility
    public PathResult findExplorationPath(
      Point start,
      LocalMap map,
      Map<Point, Double> heatMap
    ) {
      return findExplorationPath(start, map, heatMap, new SearchParams());
    }
  }

  public PathResult getActivePath(String agName) {
    return activePaths.get(agName);
  }

  public void triggerPathRecompute(
    String agName,
    Point startPoint,
    LocalMap map,
    RecomputeReason reason
  ) {
    PathResult newPath = findNewPath(agName, startPoint, map, reason);
    if (newPath.success) {
      activePaths.put(agName, newPath);
      pathStartTimes.put(agName, System.currentTimeMillis());
    } else {
      activePaths.remove(agName);
    }
  }

  private void addStuckPenalty(Point stuckPoint) {
    // Create a larger penalty area when stuck
    for (int dx = -3; dx <= 3; dx++) {
      for (int dy = -3; dy <= 3; dy++) {
        Point p = new Point(stuckPoint.x + dx, stuckPoint.y + dy);
        // Higher penalty in immediate vicinity
        double distance = Math.sqrt(dx * dx + dy * dy);
        double penalty = INITIAL_HEAT * (3.0 - (distance / 4.0));
        heatMap.put(p, Math.max(heatMap.getOrDefault(p, 0.0), penalty));
      }
    }
  }

  private void adjustForTraffic(String agName, Point current) {
    Point currentZone = getZone(current);
    ZoneInfo info = zoneMemory.get(currentZone);

    if (info != null && info.visitingAgents.size() > 1) {
      // Find zones with fewer agents
      Map<Point, Double> zoneScores = new HashMap<>();
      for (int dx = -3; dx <= 3; dx++) {
        for (int dy = -3; dy <= 3; dy++) {
          Point zonePoint = new Point(currentZone.x + dx, currentZone.y + dy);
          ZoneInfo zoneInfo = zoneMemory.get(zonePoint);
          if (zoneInfo == null || zoneInfo.visitingAgents.size() <= 1) {
            zoneScores.put(zonePoint, getAdjacentZonesScore(zonePoint, agName));
          }
        }
      }

      // Update heat map to prefer less crowded zones
      zoneScores.forEach(
        (zone, score) -> {
          Point center = new Point(zone.x * ZONE_SIZE, zone.y * ZONE_SIZE);
          updateHeatInRadius(center, 1.0 - score); // Lower heat for better scores
        }
      );
    }
  }

  private void resetExplorationState(String agName) {
    // Clear any stuck penalties
    heatMap.replaceAll((k, v) -> Math.min(v, INITIAL_HEAT));

    // Reset path timeout
    pathStartTimes.remove(agName);

    // Get current position from active path
    PathResult currentPath = activePaths.get(agName);
    if (currentPath != null && currentPath.startPosition != null) {
      updateHeatInRadius(currentPath.startPosition, 0.5); // Reduce heat in current area
    }
  }

  private void updateHeatInRadius(Point center, double intensity) {
    for (int dx = -HEAT_RADIUS; dx <= HEAT_RADIUS; dx++) {
      for (int dy = -HEAT_RADIUS; dy <= HEAT_RADIUS; dy++) {
        Point p = new Point(center.x + dx, center.y + dy);
        double distance = Math.sqrt(dx * dx + dy * dy);
        if (distance <= HEAT_RADIUS) {
          double heatValue = intensity * Math.exp(-distance / HEAT_RADIUS);
          heatMap.merge(p, heatValue, (old, new_) -> Math.min(1.0, old + new_));
        }
      }
    }
  }

  private void addOscillationPenalty(Point oscillationPoint) {
    // Add very high heat to oscillation area to strongly discourage revisiting
    for (int dx = -1; dx <= 1; dx++) {
      for (int dy = -1; dy <= 1; dy++) {
        Point p = new Point(oscillationPoint.x + dx, oscillationPoint.y + dy);
        heatMap.put(p, INITIAL_HEAT * 3); // Triple the normal heat
      }
    }

    // Also add moderate heat to a wider area
    for (int dx = -3; dx <= 3; dx++) {
      for (int dy = -3; dy <= 3; dy++) {
        if (Math.abs(dx) <= 1 && Math.abs(dy) <= 1) continue; // Skip inner area
        Point p = new Point(oscillationPoint.x + dx, oscillationPoint.y + dy);
        heatMap.merge(p, INITIAL_HEAT, Double::max);
      }
    }
  }

  public String getNextDirection(
    String agName,
    Point currentPos,
    List<String> availableDirections,
    LocalMap map
  ) {
    // First try to use existing path if valid
    PathResult currentPath = getActivePath(agName);
    if (currentPath != null && !currentPath.directions.isEmpty()) {
      String nextInLine = currentPath.directions.get(0);
      if (
        availableDirections.contains(nextInLine) &&
        isPathStillValid(currentPath, currentPos, map)
      ) {
        currentPath.directions.remove(0); // Remove the used direction
        return nextInLine;
      }
    }

    // If we get here, we need to compute a new path
    return computeNewPath(agName, currentPos, availableDirections, map);
  }

  private boolean isPathStillValid(
    PathResult path,
    Point currentPos,
    LocalMap map
  ) {
    if (path == null || path.directions.isEmpty()) {
      return false;
    }

    // Check if path start position matches current position
    if (!currentPos.equals(path.startPosition)) {
      return false;
    }

    // Check if target conditions still valid (not blocked, still exists, etc)
    if (path.targetType != null && path.targetPosition != null) {
      Point target = findNearestTarget(map, currentPos, path.targetType);
      if (target == null || !target.equals(path.targetPosition)) {
        return false;
      }
    }

    return true;
  }

  private String computeNewPath(
    String agName,
    Point currentPos,
    List<String> availableDirections,
    LocalMap map
  ) {
    // Get exploration scores for different areas
    Map<Point, Double> explorationScores = calculateExplorationScores(
      map,
      currentPos
    );

    // Get heatmap of visited areas
    Map<Point, Double> heatmap = map.getVisitedHeatmap();

    // Find best direction considering multiple factors
    String bestDirection = null;
    double bestScore = Double.NEGATIVE_INFINITY;

    for (String direction : availableDirections) {
      Point nextPos = calculateNextPosition(currentPos, direction);

      // Skip if position is invalid
      if (map.isOutOfBounds(nextPos) || map.hasObstacle(nextPos)) {
        continue;
      }

      double score = evaluateDirection(
        nextPos,
        explorationScores,
        heatmap,
        map
      );

      if (score > bestScore) {
        bestScore = score;
        bestDirection = direction;
      }
    }

    if (bestDirection != null) {
      Point nextPosition = calculateNextPosition(currentPos, bestDirection);
      // Create new path and store it
      PathResult newPath = new PathResult(
        currentPos,
        Collections.singletonList(bestDirection),
        nextPosition,
        null
      );
      setActivePath(agName, newPath);
    }

    return bestDirection;
  }

  private double evaluateDirection(
    Point nextPos,
    Map<Point, Double> explorationScores,
    Map<Point, Double> heatmap,
    LocalMap map
  ) {
    double score = 0.0;

    // Exploration potential (higher is better)
    score += explorationScores.getOrDefault(nextPos, 0.0) * EXPLORATION_WEIGHT;

    // Visited frequency (lower is better)
    double heatValue = heatmap.getOrDefault(nextPos, 0.0);
    score -= heatValue * HEATMAP_WEIGHT;

    // Distance from boundaries (higher is better)
    double boundaryDistance = calculateBoundaryDistance(nextPos, map);
    score += boundaryDistance * BOUNDARY_WEIGHT;

    // Add bonus for unexplored areas
    if (!heatmap.containsKey(nextPos)) {
      score += UNEXPLORED_BONUS;
    }

    return score;
  }

  private Map<Point, Double> calculateExplorationScores(
    LocalMap map,
    Point currentPos
  ) {
    Map<Point, Double> scores = new HashMap<>();
    int radius = 5; // Look ahead radius

    for (int dx = -radius; dx <= radius; dx++) {
      for (int dy = -radius; dy <= radius; dy++) {
        Point p = new Point(currentPos.x + dx, currentPos.y + dy);

        if (map.isOutOfBounds(p) || map.hasObstacle(p)) {
          continue;
        }

        // Calculate exploration potential based on distance and unexplored neighbors
        double distanceScore = 1.0 / (1.0 + manhattanDistance(currentPos, p));
        double unexploredScore = calculateUnexploredNeighborScore(p, map);

        scores.put(p, distanceScore * unexploredScore);
      }
    }

    return scores;
  }

  private double calculateUnexploredNeighborScore(Point p, LocalMap map) {
    int unexploredCount = 0;
    int totalNeighbors = 0;

    for (int dx = -1; dx <= 1; dx++) {
      for (int dy = -1; dy <= 1; dy++) {
        if (dx == 0 && dy == 0) continue;

        Point neighbor = new Point(p.x + dx, p.y + dy);
        if (!map.isOutOfBounds(neighbor)) {
          totalNeighbors++;
          if (!map.isExplored(neighbor)) {
            unexploredCount++;
          }
        }
      }
    }

    return totalNeighbors > 0 ? (double) unexploredCount / totalNeighbors : 0.0;
  }

  private double calculateBoundaryDistance(Point pos, LocalMap map) {
    Map<String, Point> boundaries = map.getConfirmedBoundariesPositions();
    if (boundaries.isEmpty()) {
      return 1.0; // Maximum score if no boundaries known
    }

    double minDistance = Double.MAX_VALUE;
    for (Point boundaryPos : boundaries.values()) {
      minDistance = Math.min(minDistance, manhattanDistance(pos, boundaryPos));
    }

    return Math.min(1.0, minDistance / 10.0); // Normalize to [0,1]
  }

  private int manhattanDistance(Point p1, Point p2) {
    return Math.abs(p1.x - p2.x) + Math.abs(p1.y - p2.y);
  }

  // Constants for scoring weights
  private static final double EXPLORATION_WEIGHT = 0.4;
  private static final double HEATMAP_WEIGHT = 0.3;
  private static final double BOUNDARY_WEIGHT = 0.2;

  private void setActivePath(String agName, PathResult path) {
    activePaths.put(agName, path);
  }

  private Point findNearestTarget(
    LocalMap map,
    Point currentPos,
    String targetType
  ) {
    // Implementation needed
    // This should return the nearest target of the specified type
    return null;
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

  private PathResult findNewPath(
    String agName,
    Point current,
    LocalMap map,
    RecomputeReason reason
  ) {
    // For normal exploration (no recompute reason), just find a good path
    if (reason == null) {
      ExplorationSearch.SearchParams params = new ExplorationSearch.SearchParams();
      params.maxDepth = 10;
      params.unexploredWeight = 0.7;
      params.heatWeight = 0.2;
      params.distanceWeight = 0.1;

      return explorationSearch.findExplorationPath(
        current,
        map,
        heatMap,
        params
      );
    }

    // For recompute cases, use specialized parameters
    ExplorationSearch.SearchParams params = new ExplorationSearch.SearchParams();
    switch (reason) {
      case STUCK:
        params.maxDepth = 8;
        params.unexploredWeight = 0.5;
        params.heatWeight = 0.4; // Higher weight to avoid stuck areas
        params.distanceWeight = 0.1;
        break;
      case OSCILLATION:
        params.maxDepth = 12;
        params.unexploredWeight = 0.6;
        params.heatWeight = 0.3;
        params.distanceWeight = 0.1;
        break;
      case EXPLORATION_TIMEOUT:
        params.maxDepth = 15;
        params.unexploredWeight = 0.8; // Higher weight for unexplored areas
        params.heatWeight = 0.1;
        params.distanceWeight = 0.1;
        break;
      default:
        params.maxDepth = 10;
        params.unexploredWeight = 0.7;
        params.heatWeight = 0.2;
        params.distanceWeight = 0.1;
    }

    return explorationSearch.findExplorationPath(current, map, heatMap, params);
  }
}
