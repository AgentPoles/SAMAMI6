package jason.eis.movements;

import jason.eis.LocalMap;
import jason.eis.Point;
import java.util.*;
import java.util.concurrent.ConcurrentHashMap;

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
  }

  public static class PathResult {
    public final List<String> directions;
    public final List<Point> points;
    public final boolean success;
    public final RecomputeReason reason;

    public PathResult(
      List<String> directions,
      List<Point> points,
      boolean success,
      RecomputeReason reason
    ) {
      this.directions = directions;
      this.points = points;
      this.success = success;
      this.reason = reason;
    }

    // Add constructor for backward compatibility
    public PathResult(
      List<String> directions,
      List<Point> points,
      boolean success
    ) {
      this(directions, points, success, null);
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
    // Check existing path first (fast path)
    PathResult existingPath = activePaths.get(agName);
    if (existingPath != null && !existingPath.directions.isEmpty()) {
      String nextDir = existingPath.directions.get(0);
      if (safeDirections.contains(nextDir)) {
        // Update path by removing the first step
        List<String> remainingDirections = new ArrayList<>(
          existingPath.directions
        );
        remainingDirections.remove(0);
        List<Point> remainingPoints = new ArrayList<>(existingPath.points);
        if (!remainingPoints.isEmpty()) {
          remainingPoints.remove(0);
        }
        PathResult updatedPath = new PathResult(
          remainingDirections,
          remainingPoints,
          true
        );
        activePaths.put(agName, updatedPath);
        return updatedPath;
      }
    }

    // Only calculate new path if we're near unexplored area
    if (hasNearbyUnexplored(current, map)) {
      return findNewPath(
        agName,
        current,
        map,
        RecomputeReason.EXPLORATION_TIMEOUT
      );
    }

    return new PathResult(new ArrayList<>(), new ArrayList<>(), false);
  }

  private PathResult findNewPath(
    String agName,
    Point current,
    LocalMap map,
    RecomputeReason reason
  ) {
    // Initialize default parameters
    ExplorationSearch.SearchParams params = new ExplorationSearch.SearchParams();

    // Only adjust parameters if reason is not null
    if (reason != null) {
      switch (reason) {
        case PATH_BLOCKED:
          params.maxDepth = 14;
          params.unexploredWeight = 0.7;
          params.heatWeight = 0.2;
          params.distanceWeight = 0.1;
          // Add extra heat to blocked area
          updateHeatInRadius(current);
          break;
        case STUCK:
          params.maxDepth = 10;
          params.unexploredWeight = 0.8;
          params.heatWeight = 0.1;
          params.distanceWeight = 0.1;
          // Heavily penalize the stuck area
          addStuckPenalty(current);
          break;
        case HIGH_TRAFFIC:
          params.maxDepth = 14;
          params.unexploredWeight = 0.5;
          params.heatWeight = 0.3;
          params.distanceWeight = 0.2;
          // Prefer less crowded zones
          adjustForTraffic(agName, current);
          break;
        case EXPLORATION_TIMEOUT:
          params.maxDepth = 14;
          params.unexploredWeight = 0.6;
          params.heatWeight = 0.2;
          params.distanceWeight = 0.2;
          // Reset exploration parameters
          resetExplorationState(agName);
          break;
      }
    }

    return explorationSearch.findExplorationPath(current, map, heatMap, params);
  }

  private boolean hasNearbyUnexplored(Point current, LocalMap map) {
    // Quick check in small radius
    for (int d = 1; d <= 3; d++) {
      for (int dx = -d; dx <= d; dx++) {
        for (int dy = -d; dy <= d; dy++) {
          if (Math.abs(dx) + Math.abs(dy) != d) continue;
          Point p = new Point(current.x + dx, current.y + dy);
          // Check if point is unexplored using available map methods
          if (!map.isForbidden(p) && !map.hasObstacle(p)) {
            return true;
          }
        }
      }
    }
    return false;
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
              current.depth + 1
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
        return new PathResult(new ArrayList<>(), new ArrayList<>(), false);
      }

      return reconstructPath(bestNode);
    }

    private double calculateDirectionScore(
      Point pos,
      LocalMap map,
      Map<Point, Double> heatMap,
      int depth
    ) {
      // Count unexplored neighbors
      int unexploredCount = countUnexploredNeighbors(pos, map);
      double unexploredScore = unexploredCount / 4.0; // Normalize to 0-1

      // Heat map penalty (lower is better)
      double heatPenalty = heatMap.getOrDefault(pos, 0.0);

      // Distance penalty
      double distancePenalty = depth / (double) MAX_DEPTH;

      return (
        (unexploredScore * UNEXPLORED_WEIGHT) -
        (heatPenalty * HEAT_WEIGHT) -
        (distancePenalty * DISTANCE_WEIGHT)
      );
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
      return calculateDirectionScore(node.position, map, heatMap, node.depth);
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
      List<Point> points = new ArrayList<>();

      // Work backwards from target to start
      ExplorationNode current = node;
      while (current.parent != null) {
        directions.add(0, current.direction);
        points.add(0, current.position);
        current = current.parent;
      }

      return new PathResult(directions, points, true);
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
    // Add high heat to stuck area to avoid it
    for (int dx = -2; dx <= 2; dx++) {
      for (int dy = -2; dy <= 2; dy++) {
        Point p = new Point(stuckPoint.x + dx, stuckPoint.y + dy);
        heatMap.put(p, INITIAL_HEAT * 2);
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

    // Increase weight for unexplored areas
    Point current = activePaths.get(agName).points.get(0);
    Point currentZone = getZone(current);
    updateHeatInRadius(current, 0.5); // Reduce heat in current area
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
}
