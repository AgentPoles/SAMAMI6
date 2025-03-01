package jason.eis.movements;

import jason.eis.LocalMap;
import jason.eis.Point;
import java.util.*;
import java.util.concurrent.ConcurrentHashMap;
import java.util.logging.Logger;

public class Search {
  private static final Logger logger = Logger.getLogger(Search.class.getName());
  private static final boolean DEBUG = true;
  private static final int MAX_ITERATIONS = 1000; // Reduced from previous value
  private static final int MAX_PATH_LENGTH = 100; // Maximum acceptable path length
  private static final double HEURISTIC_WEIGHT = 1.2; // Slightly favor exploration
  private static final int DYNAMIC_OBSTACLE_BUFFER = 2; // Buffer zone around dynamic obstacles
  private static final double MAX_PATH_COST = 1000.0; // Maximum reasonable path cost

  // Search types
  public enum TargetType {
    DISPENSER,
    BLOCK,
    GOAL,
    ATTACHMENT_POINT, // For when we need to path to specific attachment positions
  }

  // Direction definitions
  private static final String[] DIRECTIONS = { "n", "e", "s", "w" };
  private static final Map<String, Point> DIRECTION_VECTORS = new HashMap<>();

  static {
    try {
      DIRECTION_VECTORS.put("n", new Point(0, -1));
      DIRECTION_VECTORS.put("e", new Point(1, 0));
      DIRECTION_VECTORS.put("s", new Point(0, 1));
      DIRECTION_VECTORS.put("w", new Point(-1, 0));
    } catch (Exception e) {
      logger.severe(
        "Failed to initialize direction vectors: " + e.getMessage()
      );
    }
  }

  private static class JumpPoint {
    Point position;
    JumpPoint parent;
    double gCost;
    double fCost;
    String direction;

    JumpPoint(
      Point position,
      JumpPoint parent,
      double gCost,
      double hCost,
      String direction
    ) {
      this.position = position;
      this.parent = parent;
      this.gCost = gCost;
      this.fCost = gCost + hCost;
      this.direction = direction;
    }
  }

  public static class PathResult {
    public final List<String> directions;
    public final List<Point> points;
    public final boolean success;

    PathResult(List<String> dirs, List<Point> pts, boolean success) {
      this.directions = Collections.unmodifiableList(new ArrayList<>(dirs));
      this.points = Collections.unmodifiableList(new ArrayList<>(pts));
      this.success = success;
    }
  }

  private final ObstacleManager obstacleManager;

  // Cache recently calculated paths
  private final Map<PathKey, CachedPath> pathCache = new ConcurrentHashMap<>();
  private static final int CACHE_TTL = 5000; // 5 seconds cache lifetime
  private static final int MAX_CACHE_SIZE = 100;

  private static class PathKey {
    final Point start;
    final Point target;
    final int size;
    final String blockDir;

    PathKey(Point start, Point target, int size, String blockDir) {
      this.start = start;
      this.target = target;
      this.size = size;
      this.blockDir = blockDir;
    }

    @Override
    public boolean equals(Object o) {
      if (this == o) return true;
      if (!(o instanceof PathKey)) return false;
      PathKey key = (PathKey) o;
      return (
        size == key.size &&
        Objects.equals(start, key.start) &&
        Objects.equals(target, key.target) &&
        Objects.equals(blockDir, key.blockDir)
      );
    }

    @Override
    public int hashCode() {
      return Objects.hash(start, target, size, blockDir);
    }
  }

  private static class CachedPath {
    final PathResult result;
    final long timestamp;

    CachedPath(PathResult result) {
      this.result = result;
      this.timestamp = System.currentTimeMillis();
    }

    boolean isValid() {
      return System.currentTimeMillis() - timestamp < CACHE_TTL;
    }
  }

  public Search() {
    this.obstacleManager = new ObstacleManager();
  }

  public PathResult findPath(
    Point start,
    Point target,
    LocalMap map,
    TargetType targetType,
    int agentSize,
    String blockDirection
  ) {
    try {
      // Check cache first
      PathKey key = new PathKey(start, target, agentSize, blockDirection);
      CachedPath cached = pathCache.get(key);
      if (cached != null && cached.isValid()) {
        return cached.result;
      }

      // Validate inputs
      if (!validateInputs(start, target, map, targetType, agentSize)) {
        return createEmptyPathResult("Invalid input parameters");
      }

      // Quick reachability check
      if (!isTargetReachable(target, map, agentSize, blockDirection)) {
        return createEmptyPathResult("Target unreachable");
      }

      // Use simplified A* for distant targets
      int manhattanDistance =
        Math.abs(target.x - start.x) + Math.abs(target.y - start.y);
      if (manhattanDistance > MAX_PATH_LENGTH) {
        return findApproximatePath(
          start,
          target,
          map,
          agentSize,
          blockDirection
        );
      }

      // Standard A* with optimizations
      PathResult result = findOptimizedPath(
        start,
        target,
        map,
        agentSize,
        blockDirection
      );

      // Cache the result
      if (result.success) {
        cacheResult(key, result);
      }

      return result;
    } catch (Exception e) {
      logger.severe("Error in pathfinding: " + e.getMessage());
      return createEmptyPathResult("Pathfinding error");
    }
  }

  private boolean validateInputs(
    Point start,
    Point target,
    LocalMap map,
    TargetType targetType,
    int agentSize
  ) {
    if (start == null || target == null || map == null || targetType == null) {
      logger.warning(
        String.format(
          "Invalid inputs: start=%s, target=%s, map=%s, type=%s",
          start,
          target,
          map != null ? "valid" : "null",
          targetType
        )
      );
      return false;
    }

    if (agentSize <= 0) {
      logger.warning("Invalid agent size: " + agentSize);
      return false;
    }

    // Check if target is forbidden (boundary or obstacle)
    if (map.isForbidden(target)) {
      if (DEBUG) {
        logger.warning(
          String.format(
            "Target position %s is forbidden (boundary or obstacle)",
            target
          )
        );
      }
      return false;
    }

    // Check if start position has obstacles
    if (map.hasObstacle(start)) {
      logger.warning(
        String.format("Start position %s contains an obstacle", start)
      );
      return false;
    }

    return true;
  }

  private boolean isTargetReachable(
    Point target,
    LocalMap map,
    int agentSize,
    String blockDirection
  ) {
    try {
      // Check if target has static obstacles
      if (map.isStaticObstacle(target)) {
        if (DEBUG) {
          logger.info("Target position has static obstacle: " + target);
        }
        return false;
      }

      // Check if target is near boundary
      if (map.isNearBoundary(target)) {
        if (DEBUG) {
          logger.info("Target is too close to boundary: " + target);
        }
        return false;
      }

      // Check surrounding area for obstacles
      Set<Point> nearbyObstacles = map.getObstaclesInRange(target, agentSize);
      if (!nearbyObstacles.isEmpty()) {
        if (DEBUG) {
          logger.info("Obstacles found near target: " + nearbyObstacles);
        }
        return false;
      }

      return true;
    } catch (Exception e) {
      logger.warning("Error checking target reachability: " + e.getMessage());
      return false;
    }
  }

  private PathResult findOptimizedPath(
    Point start,
    Point target,
    LocalMap map,
    int agentSize,
    String blockDirection
  ) {
    PriorityQueue<JumpPoint> openSet = new PriorityQueue<>(
      Comparator.comparingDouble(p -> p.fCost)
    );
    Map<Point, JumpPoint> visited = new HashMap<>();

    // Initial point
    JumpPoint startPoint = new JumpPoint(
      start,
      null,
      0,
      getHeuristic(start, target),
      null
    );
    openSet.add(startPoint);
    visited.put(start, startPoint);

    while (!openSet.isEmpty()) {
      JumpPoint current = openSet.poll();

      if (isCloseEnough(current.position, target, agentSize)) {
        return reconstructPath(current, map, agentSize, blockDirection);
      }

      for (String dir : DIRECTIONS) {
        Point neighbor = getNeighbor(current.position, dir);
        if (neighbor == null || map.isForbidden(neighbor)) {
          continue;
        }

        double tentativeG =
          current.gCost + getMovementCost(current.position, neighbor, map);
        JumpPoint neighborPoint = visited.get(neighbor);

        if (neighborPoint == null || tentativeG < neighborPoint.gCost) {
          double h = getHeuristic(neighbor, target);
          neighborPoint = new JumpPoint(neighbor, current, tentativeG, h, dir);
          visited.put(neighbor, neighborPoint);
          openSet.add(neighborPoint);
        }
      }
    }

    return createEmptyPathResult("No path found");
  }

  private PathResult findApproximatePath(
    Point start,
    Point target,
    LocalMap map,
    int agentSize,
    String blockDirection
  ) {
    // For distant targets, use a simplified approach with waypoints
    List<Point> waypoints = new ArrayList<>();
    Point current = start;

    while (getManhattanDistance(current, target) > MAX_PATH_LENGTH / 2) {
      // Find an intermediate point in the general direction
      Point intermediate = findIntermediatePoint(
        current,
        target,
        map,
        agentSize
      );
      if (intermediate == null) {
        return createEmptyPathResult("No valid intermediate point found");
      }
      waypoints.add(intermediate);
      current = intermediate;
    }

    // Add final target
    waypoints.add(target);

    return createWaypointPath(start, waypoints, map, agentSize, blockDirection);
  }

  private Point findIntermediatePoint(
    Point start,
    Point target,
    LocalMap map,
    int agentSize
  ) {
    int dx = target.x - start.x;
    int dy = target.y - start.y;
    int step = MAX_PATH_LENGTH / 2;

    // Try points in the general direction
    Point[] candidates = {
      new Point(start.x + (dx > 0 ? step : -step), start.y),
      new Point(start.x, start.y + (dy > 0 ? step : -step)),
      new Point(
        start.x + (dx > 0 ? step / 2 : -step / 2),
        start.y + (dy > 0 ? step / 2 : -step / 2)
      ),
    };

    for (Point candidate : candidates) {
      if (
        !map.isForbidden(candidate) && !isNearDynamicObstacles(candidate, map)
      ) {
        return candidate;
      }
    }

    return null;
  }

  private boolean isNearDynamicObstacles(Point pos, LocalMap map) {
    Map<Point, LocalMap.ObstacleInfo> dynamicObstacles = map.getDynamicObstacles();
    for (Point obstacle : dynamicObstacles.keySet()) {
      if (getManhattanDistance(pos, obstacle) < DYNAMIC_OBSTACLE_BUFFER) {
        return true;
      }
    }
    return false;
  }

  private void cacheResult(PathKey key, PathResult result) {
    // Remove old entries if cache is too large
    if (pathCache.size() >= MAX_CACHE_SIZE) {
      pathCache.entrySet().removeIf(entry -> !entry.getValue().isValid());
      if (pathCache.size() >= MAX_CACHE_SIZE) {
        // If still too large, remove oldest entries
        List<Map.Entry<PathKey, CachedPath>> entries = new ArrayList<>(
          pathCache.entrySet()
        );
        entries.sort(
          (a, b) -> Long.compare(a.getValue().timestamp, b.getValue().timestamp)
        );
        for (int i = 0; i < entries.size() / 2; i++) {
          pathCache.remove(entries.get(i).getKey());
        }
      }
    }
    pathCache.put(key, new CachedPath(result));
  }

  private boolean isCloseEnough(Point current, Point target, int agentSize) {
    return getManhattanDistance(current, target) <= Math.max(1, agentSize - 1);
  }

  private double getMovementCost(Point from, Point to, LocalMap map) {
    double baseCost = 1.0;

    // Increase cost near dynamic obstacles
    if (isNearDynamicObstacles(to, map)) {
      baseCost *= 2.0;
    }

    // Increase cost near boundaries
    if (map.isNearBoundary(to)) {
      baseCost *= 1.5;
    }

    return baseCost;
  }

  private double heuristic(Point a, Point b) {
    try {
      return Math.abs(a.x - b.x) + Math.abs(a.y - b.y);
    } catch (Exception e) {
      logger.warning("Error calculating heuristic: " + e.getMessage());
      return Double.MAX_VALUE; // Return maximum value to discourage this path
    }
  }

  private Point applyDirection(Point pos, String direction) {
    try {
      Point vector = DIRECTION_VECTORS.get(direction);
      if (vector == null) {
        logger.warning("Invalid direction: " + direction);
        return null;
      }
      return new Point(pos.x + vector.x, pos.y + vector.y);
    } catch (Exception e) {
      logger.warning("Error applying direction: " + e.getMessage());
      return null;
    }
  }

  private PathResult reconstructPath(
    JumpPoint endPoint,
    LocalMap map,
    int agentSize,
    String blockDirection
  ) {
    List<Point> points = new ArrayList<>();
    List<String> directions = new ArrayList<>();
    JumpPoint current = endPoint;

    while (current != null) {
      points.add(0, current.position);
      if (current.direction != null) {
        directions.add(0, current.direction);
      }
      current = current.parent;
    }

    return new PathResult(directions, points, true);
  }

  private PathResult createEmptyPathResult(String reason) {
    if (DEBUG) {
      logger.info("Creating empty path result: " + reason);
    }
    return new PathResult(new ArrayList<>(), new ArrayList<>(), false);
  }

  private JumpPoint createStartPoint(Point start, Point target) {
    return new JumpPoint(start, null, 0, heuristic(start, target), null);
  }

  private boolean isAtTarget(Point pos, Point target, TargetType type) {
    try {
      return pos.equals(target);
    } catch (Exception e) {
      logger.warning("Error checking target: " + e.getMessage());
      return false;
    }
  }

  private double getHeuristic(Point start, Point target) {
    return heuristic(start, target);
  }

  private List<Point> getValidNeighbors(
    Point current,
    LocalMap map,
    int agentSize,
    String blockDirection,
    Point target
  ) {
    List<Point> neighbors = new ArrayList<>();
    for (String direction : DIRECTIONS) {
      Point nextPos = applyDirection(current, direction);
      if (
        nextPos != null && !isWalkable(nextPos, map, agentSize, blockDirection)
      ) {
        neighbors.add(nextPos);
      }
    }
    return neighbors;
  }

  private boolean isWalkable(
    Point pos,
    LocalMap map,
    int agentSize,
    String blockDirection
  ) {
    try {
      return (
        obstacleManager
          .filterDirections(
            Arrays.asList(DIRECTIONS),
            map,
            pos,
            agentSize,
            blockDirection
          )
          .size() >
        0
      );
    } catch (Exception e) {
      logger.warning(
        "Error checking walkable position " + pos + ": " + e.getMessage()
      );
      return false;
    }
  }

  private double getManhattanDistance(Point a, Point b) {
    return Math.abs(a.x - b.x) + Math.abs(a.y - b.y);
  }

  private PathResult createWaypointPath(
    Point start,
    List<Point> waypoints,
    LocalMap map,
    int agentSize,
    String blockDirection
  ) {
    List<String> directions = new ArrayList<>();
    List<Point> points = new ArrayList<>();

    Point current = start;
    for (Point waypoint : waypoints) {
      PathResult path = findPath(
        current,
        waypoint,
        map,
        TargetType.BLOCK,
        agentSize,
        blockDirection
      );
      if (!path.success) {
        return createEmptyPathResult("Path to waypoint failed");
      }
      directions.addAll(path.directions);
      points.addAll(path.points);
      current = waypoint;
    }

    return new PathResult(directions, points, true);
  }

  private Point getNeighbor(Point pos, String direction) {
    Point nextPos = applyDirection(pos, direction);
    return nextPos;
  }
}
