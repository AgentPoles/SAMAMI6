package jason.eis.movements;

import jason.eis.LocalMap;
import jason.eis.Point;
import java.util.*;
import java.util.concurrent.ConcurrentHashMap;
import java.util.logging.Logger;

public class Search {
  private static final Logger logger = Logger.getLogger(Search.class.getName());
  private static final boolean DEBUG = false;

  // Iteration limits for different search types
  private static final int QUICK_PATH_ITERATIONS = 50;
  private static final int FALLBACK_ITERATIONS = 25;
  private static final int NEARBY_ITERATIONS = 15;
  private static final int MINIMUM_PROGRESS = 5;

  // Existing constants
  private static final String[] DIRECTIONS = { "n", "e", "s", "w" };
  private static final Map<String, Point> DIRECTION_VECTORS = new HashMap<>();
  private final ObstacleManager obstacleManager;
  private static final double PROGRESS_WEIGHT = 0.8; // Weight for progress towards target
  private static final double CLEARANCE_WEIGHT = 0.2; // Weight for clear space

  static {
    DIRECTION_VECTORS.put("n", new Point(0, -1));
    DIRECTION_VECTORS.put("e", new Point(1, 0));
    DIRECTION_VECTORS.put("s", new Point(0, 1));
    DIRECTION_VECTORS.put("w", new Point(-1, 0));
  }

  public enum TargetType {
    DISPENSER,
    BLOCK,
    GOAL,
    ATTACHMENT_POINT,
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
      if (!validateInputs(start, target, map, targetType, agentSize)) {
        return new PathResult(new ArrayList<>(), new ArrayList<>(), false);
      }

      int distance = (int) getManhattanDistance(start, target);

      // For very close targets, use quick direct path
      if (distance <= 10) {
        PathResult quickPath = findQuickPath(
          start,
          target,
          map,
          agentSize,
          blockDirection,
          NEARBY_ITERATIONS
        );
        if (quickPath.success) return quickPath;
      }

      // Try to find complete path with limited iterations
      PathResult completePath = findOptimalPath(
        start,
        target,
        map,
        agentSize,
        blockDirection,
        QUICK_PATH_ITERATIONS
      );
      if (completePath.success) return completePath;

      // If complete path fails, try partial progress
      return findProgressivePath(
        start,
        target,
        map,
        agentSize,
        blockDirection,
        FALLBACK_ITERATIONS
      );
    } catch (Exception e) {
      logger.warning("Path finding error: " + e.getMessage());
      return new PathResult(new ArrayList<>(), new ArrayList<>(), false);
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

    if (map.hasObstacle(start)) {
      logger.warning(
        String.format("Start position %s contains an obstacle", start)
      );
      return false;
    }

    return true;
  }

  private boolean isNearby(Point start, Point target) {
    return Math.abs(start.x - target.x) + Math.abs(start.y - target.y) <= 10;
  }

  private PathResult findQuickPath(
    Point start,
    Point target,
    LocalMap map,
    int agentSize,
    String blockDirection,
    int maxIterations
  ) {
    List<String> directions = new ArrayList<>();
    List<Point> points = new ArrayList<>();
    Point current = start;

    int iterations = 0;
    while (!current.equals(target) && iterations < maxIterations) {
      String direction = getNextDirection(
        current,
        target,
        map,
        agentSize,
        blockDirection
      );
      if (direction == null) break;

      Point next = getNextPoint(current, direction);
      if (
        !obstacleManager
          .filterDirections(
            "search",
            Collections.singletonList(direction),
            map,
            current,
            agentSize,
            blockDirection,
            null,
            null
          )
          .contains(direction)
      ) {
        break;
      }

      points.add(next);
      directions.add(direction);
      current = next;
      iterations++;
    }

    boolean success = current.equals(target);
    return new PathResult(directions, points, success);
  }

  private PathResult findOptimalPath(
    Point start,
    Point target,
    LocalMap map,
    int agentSize,
    String blockDirection,
    int maxIterations
  ) {
    SearchState state = new SearchState();
    Queue<SearchNode> queue = new PriorityQueue<>(
      Comparator.comparingDouble(n -> n.fScore)
    );
    Set<Point> visited = new HashSet<>();

    queue.offer(
      new SearchNode(start, null, 0, getManhattanDistance(start, target))
    );

    int iterations = 0;
    while (!queue.isEmpty() && iterations < maxIterations) {
      SearchNode current = queue.poll();

      if (current.position.equals(target)) {
        return reconstructPath(current);
      }

      if (visited.contains(current.position)) continue;
      visited.add(current.position);

      // Update best partial path if this is better
      updateBestPartialPath(current, target, state);

      // Try all possible directions
      for (String dir : DIRECTIONS) {
        if (
          !isValidMove(current.position, dir, map, agentSize, blockDirection)
        ) continue;

        Point next = getNextPoint(current.position, dir);
        double newG = current.gScore + 1;
        double newF = newG + getManhattanDistance(next, target);

        queue.offer(new SearchNode(next, current, newG, newF, dir));
      }

      iterations++;
    }

    // Return best partial path if we found something useful
    if (state.bestProgress >= MINIMUM_PROGRESS) {
      return new PathResult(state.bestDirections, state.bestPoints, true);
    }

    return new PathResult(new ArrayList<>(), new ArrayList<>(), false);
  }

  private PathResult findProgressivePath(
    Point start,
    Point target,
    LocalMap map,
    int agentSize,
    String blockDirection,
    int maxIterations
  ) {
    List<String> directions = new ArrayList<>();
    List<Point> points = new ArrayList<>();
    Point current = start;
    double initialDist = getManhattanDistance(start, target);

    int iterations = 0;
    while (!current.equals(target) && iterations < maxIterations) {
      String bestDir = findBestDirection(
        current,
        target,
        map,
        agentSize,
        blockDirection
      );
      if (bestDir == null) break;

      Point next = getNextPoint(current, bestDir);
      points.add(next);
      directions.add(bestDir);
      current = next;

      // Stop if we're not making progress
      if (
        iterations > MINIMUM_PROGRESS &&
        getManhattanDistance(current, target) >= initialDist
      ) {
        break;
      }

      iterations++;
    }

    return new PathResult(directions, points, !points.isEmpty());
  }

  private String findBestDirection(
    Point current,
    Point target,
    LocalMap map,
    int agentSize,
    String blockDirection
  ) {
    String bestDir = null;
    double bestScore = Double.NEGATIVE_INFINITY;

    for (String dir : DIRECTIONS) {
      if (!isValidMove(current, dir, map, agentSize, blockDirection)) continue;

      Point next = getNextPoint(current, dir);
      double score = scorePosition(next, target, map);

      if (score > bestScore) {
        bestScore = score;
        bestDir = dir;
      }
    }

    return bestDir;
  }

  private double scorePosition(Point pos, Point target, LocalMap map) {
    double distanceScore = -getManhattanDistance(pos, target);
    int clearSpace = countClearDirections(pos, map);
    return distanceScore + (clearSpace * 0.1); // Weight clear space less than distance
  }

  private static class SearchState {
    List<String> bestDirections = new ArrayList<>();
    List<Point> bestPoints = new ArrayList<>();
    double bestScore = Double.NEGATIVE_INFINITY;
    int bestProgress = 0;
  }

  private static class SearchNode {
    Point position;
    SearchNode parent;
    String direction;
    double gScore;
    double fScore;

    SearchNode(Point pos, SearchNode parent, double g, double f) {
      this(pos, parent, g, f, null);
    }

    SearchNode(Point pos, SearchNode parent, double g, double f, String dir) {
      this.position = pos;
      this.parent = parent;
      this.gScore = g;
      this.fScore = f;
      this.direction = dir;
    }
  }

  private String getNextDirection(
    Point current,
    Point target,
    LocalMap map,
    int agentSize,
    String blockDirection
  ) {
    int dx = target.x - current.x;
    int dy = target.y - current.y;

    List<String> possibleDirs = new ArrayList<>();
    if (Math.abs(dx) > Math.abs(dy)) {
      possibleDirs.add(dx > 0 ? "e" : "w");
      possibleDirs.add(dy > 0 ? "s" : "n");
    } else {
      possibleDirs.add(dy > 0 ? "s" : "n");
      possibleDirs.add(dx > 0 ? "e" : "w");
    }

    List<String> validDirs = obstacleManager.filterDirections(
      "search",
      possibleDirs,
      map,
      current,
      agentSize,
      blockDirection,
      null,
      null
    );

    return validDirs.isEmpty() ? null : validDirs.get(0);
  }

  private Point getNextPoint(Point current, String direction) {
    Point vector = DIRECTION_VECTORS.get(direction);
    if (vector == null) return current;
    return new Point(current.x + vector.x, current.y + vector.y);
  }

  private int countClearDirections(Point pos, LocalMap map) {
    int clear = 0;
    for (String dir : DIRECTIONS) {
      Point next = getNextPoint(pos, dir);
      if (!map.isForbidden(next)) {
        clear++;
      }
    }
    return clear;
  }

  private double getManhattanDistance(Point a, Point b) {
    return Math.abs(a.x - b.x) + Math.abs(a.y - b.y);
  }

  private boolean isValidMove(
    Point current,
    String direction,
    LocalMap map,
    int agentSize,
    String blockDirection
  ) {
    Point next = getNextPoint(current, direction);
    return !map.isForbidden(next) && !map.hasObstacle(next);
  }

  private void updateBestPartialPath(
    SearchNode current,
    Point target,
    SearchState state
  ) {
    PathResult path = reconstructPath(current);
    if (current.position.equals(target)) {
      state.bestDirections = path.directions;
      state.bestPoints = path.points;
      state.bestProgress = 0;
    } else {
      state.bestProgress++;
    }
  }

  private PathResult reconstructPath(SearchNode node) {
    List<String> directions = new ArrayList<>();
    List<Point> points = new ArrayList<>();

    SearchNode current = node;
    while (current != null && current.parent != null) {
      if (current.direction != null) {
        directions.add(0, current.direction);
        points.add(0, current.position);
      }
      current = current.parent;
    }

    return new PathResult(directions, points, true);
  }
}
