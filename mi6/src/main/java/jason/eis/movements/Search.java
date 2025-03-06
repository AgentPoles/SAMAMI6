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

  public enum PathRecomputeReason {
    STUCK,
    OSCILLATION,
    PATH_TIMEOUT,
    BOUNDARY_CONSTRAINT,
    OBSTACLE_CONSTRAINT,
    DEFAULT,
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
    String blockDirection,
    PathRecomputeReason reason
  ) {
    try {
      if (!validateInputs(start, target, map, targetType, agentSize)) {
        logger.warning(
          String.format(
            "Invalid inputs for path finding - start: %s, target: %s, type: %s, size: %d",
            start,
            target,
            targetType,
            agentSize
          )
        );
        return null;
      }

      debug("Finding path from %s to %s, reason: %s", start, target, reason);

      PathResult result = null;
      try {
        switch (reason) {
          case STUCK:
            result =
              handleStuckCase(start, target, map, agentSize, blockDirection);
            break;
          case OSCILLATION:
            result =
              handleOscillationCase(
                start,
                target,
                map,
                agentSize,
                blockDirection
              );
            break;
          case PATH_TIMEOUT:
            result =
              handleTimeoutCase(start, target, map, agentSize, blockDirection);
            break;
          default:
            result =
              findDefaultPath(start, target, map, agentSize, blockDirection);
        }
      } catch (Exception e) {
        logger.warning(
          String.format(
            "Error in path finding for reason %s: %s",
            reason,
            e.getMessage()
          )
        );
        return null;
      }

      if (result == null || !result.success || result.directions.isEmpty()) {
        debug(
          "No valid path found from %s to %s with reason %s",
          start,
          target,
          reason
        );
        return null;
      }

      return result;
    } catch (Exception e) {
      logger.severe(
        String.format(
          "Critical error in findPath: %s. Start: %s, Target: %s",
          e.getMessage(),
          start,
          target
        )
      );
      return null;
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
          .filterDirections("search", map, Collections.singletonList(direction))
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
      map,
      possibleDirs
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

  private PathResult handleStuckCase(
    Point start,
    Point target,
    LocalMap map,
    int agentSize,
    String blockDirection
  ) {
    try {
      debug("Handling stuck case from %s to %s", start, target);

      Set<Point> avoidPoints = new HashSet<>();
      avoidPoints.add(start);

      // Try with increased clearance first
      PathResult alternatePath = findPathWithConstraints(
        start,
        target,
        map,
        agentSize,
        blockDirection,
        QUICK_PATH_ITERATIONS * 2,
        avoidPoints,
        true
      );

      if (isValidPath(alternatePath)) {
        debug("Found alternate path for stuck case");
        return alternatePath;
      }

      debug("Alternate path failed, trying progressive path");
      PathResult progressivePath = findProgressivePathWithAvoidance(
        start,
        target,
        map,
        agentSize,
        blockDirection,
        FALLBACK_ITERATIONS,
        avoidPoints
      );

      if (isValidPath(progressivePath)) {
        return progressivePath;
      }

      logger.warning(
        String.format(
          "Failed to find path for stuck case from %s to %s",
          start,
          target
        )
      );
      return null;
    } catch (Exception e) {
      logger.warning(
        String.format(
          "Error handling stuck case: %s. Start: %s, Target: %s",
          e.getMessage(),
          start,
          target
        )
      );
      return null;
    }
  }

  private PathResult handleOscillationCase(
    Point start,
    Point target,
    LocalMap map,
    int agentSize,
    String blockDirection
  ) {
    try {
      debug("Handling oscillation case from %s to %s", start, target);

      // Create set of points to avoid (recent positions)
      Set<Point> recentPositions = new HashSet<>();
      recentPositions.add(start);

      // Try to find path avoiding recent positions
      return findPathWithConstraints(
        start,
        target,
        map,
        agentSize,
        blockDirection,
        QUICK_PATH_ITERATIONS,
        recentPositions,
        false
      );
    } catch (Exception e) {
      logger.warning("Error handling oscillation case: " + e.getMessage());
      return findDefaultPath(start, target, map, agentSize, blockDirection);
    }
  }

  private PathResult handleTimeoutCase(
    Point start,
    Point target,
    LocalMap map,
    int agentSize,
    String blockDirection
  ) {
    try {
      debug("Handling timeout case from %s to %s", start, target);

      // For timeout, try quick path first with reduced iterations
      PathResult quickPath = findQuickPath(
        start,
        target,
        map,
        agentSize,
        blockDirection,
        NEARBY_ITERATIONS
      );

      if (quickPath.success) {
        return quickPath;
      }

      // If quick path fails, try progressive path with modified heuristic
      return findProgressivePath(
        start,
        target,
        map,
        agentSize,
        blockDirection,
        FALLBACK_ITERATIONS
      );
    } catch (Exception e) {
      logger.warning("Error handling timeout case: " + e.getMessage());
      return findDefaultPath(start, target, map, agentSize, blockDirection);
    }
  }

  private PathResult findPathWithConstraints(
    Point start,
    Point target,
    LocalMap map,
    int agentSize,
    String blockDirection,
    int maxIterations,
    Set<Point> avoidPoints,
    boolean requireMoreClearance
  ) {
    try {
      if (!validateConstrainedPathInputs(start, target, map, avoidPoints)) {
        return null;
      }

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
          PathResult path = reconstructPath(current);
          if (isValidPath(path)) {
            return path;
          }
        }

        if (
          visited.contains(current.position) ||
          avoidPoints.contains(current.position)
        ) {
          continue;
        }
        visited.add(current.position);

        updateBestPartialPath(current, target, state);

        for (String dir : DIRECTIONS) {
          try {
            if (
              !processNextDirection(
                current,
                dir,
                map,
                agentSize,
                blockDirection,
                requireMoreClearance,
                avoidPoints,
                target,
                queue
              )
            ) {
              continue;
            }
          } catch (Exception e) {
            debug("Error processing direction %s: %s", dir, e.getMessage());
          }
        }
        iterations++;
      }

      if (state.bestProgress >= MINIMUM_PROGRESS) {
        return new PathResult(state.bestDirections, state.bestPoints, true);
      }

      debug("No path found within constraints after %d iterations", iterations);
      return null;
    } catch (Exception e) {
      logger.warning(
        String.format(
          "Error in findPathWithConstraints: %s. Start: %s, Target: %s",
          e.getMessage(),
          start,
          target
        )
      );
      return null;
    }
  }

  private boolean processNextDirection(
    SearchNode current,
    String dir,
    LocalMap map,
    int agentSize,
    String blockDirection,
    boolean requireMoreClearance,
    Set<Point> avoidPoints,
    Point target,
    Queue<SearchNode> queue
  ) {
    Point next = getNextPoint(current.position, dir);

    if (
      !isValidMoveWithConstraints(
        current.position,
        dir,
        map,
        agentSize,
        blockDirection,
        requireMoreClearance
      )
    ) {
      return false;
    }

    if (avoidPoints.contains(next)) {
      return false;
    }

    double newG =
      current.gScore + getMovementCost(next, map, requireMoreClearance);
    double newF = newG + getManhattanDistance(next, target);

    queue.offer(new SearchNode(next, current, newG, newF, dir));
    return true;
  }

  private boolean validateConstrainedPathInputs(
    Point start,
    Point target,
    LocalMap map,
    Set<Point> avoidPoints
  ) {
    if (start == null || target == null || map == null) {
      logger.warning("Null inputs in constrained path finding");
      return false;
    }

    if (avoidPoints == null) {
      logger.warning("Null avoidPoints set in constrained path finding");
      return false;
    }

    if (avoidPoints.contains(target)) {
      logger.warning("Target point is in avoid set");
      return false;
    }

    return true;
  }

  private boolean isValidPath(PathResult path) {
    return path != null && path.success && !path.directions.isEmpty();
  }

  private double getMovementCost(
    Point pos,
    LocalMap map,
    boolean requireMoreClearance
  ) {
    double baseCost = 1.0;
    if (requireMoreClearance) {
      int clearDirections = countClearDirections(pos, map);
      baseCost += (4 - clearDirections) * 0.5; // Penalty for less clear space
    }
    return baseCost;
  }

  private void debug(String message, Object... args) {
    if (DEBUG) {
      logger.fine(String.format(message, args));
    }
  }

  private PathResult findDefaultPath(
    Point start,
    Point target,
    LocalMap map,
    int agentSize,
    String blockDirection
  ) {
    try {
      // Try quick path first
      PathResult quickPath = findQuickPath(
        start,
        target,
        map,
        agentSize,
        blockDirection,
        QUICK_PATH_ITERATIONS
      );

      if (isValidPath(quickPath)) {
        return quickPath;
      }

      // Try optimal path next
      PathResult optimalPath = findOptimalPath(
        start,
        target,
        map,
        agentSize,
        blockDirection,
        QUICK_PATH_ITERATIONS
      );

      if (isValidPath(optimalPath)) {
        return optimalPath;
      }

      // Fallback to progressive path
      return findProgressivePath(
        start,
        target,
        map,
        agentSize,
        blockDirection,
        FALLBACK_ITERATIONS
      );
    } catch (Exception e) {
      logger.warning("Error in findDefaultPath: " + e.getMessage());
      return null;
    }
  }

  private PathResult findProgressivePathWithAvoidance(
    Point start,
    Point target,
    LocalMap map,
    int agentSize,
    String blockDirection,
    int maxIterations,
    Set<Point> avoidPoints
  ) {
    try {
      List<String> directions = new ArrayList<>();
      List<Point> points = new ArrayList<>();
      Point current = start;
      double initialDist = getManhattanDistance(start, target);

      int iterations = 0;
      while (!current.equals(target) && iterations < maxIterations) {
        String bestDir = findBestDirectionWithAvoidance(
          current,
          target,
          map,
          agentSize,
          blockDirection,
          avoidPoints
        );

        if (bestDir == null) break;

        Point next = getNextPoint(current, bestDir);
        if (avoidPoints.contains(next)) break;

        points.add(next);
        directions.add(bestDir);
        current = next;

        if (
          iterations > MINIMUM_PROGRESS &&
          getManhattanDistance(current, target) >= initialDist
        ) {
          break;
        }

        iterations++;
      }

      return new PathResult(directions, points, !points.isEmpty());
    } catch (Exception e) {
      logger.warning(
        "Error in findProgressivePathWithAvoidance: " + e.getMessage()
      );
      return null;
    }
  }

  private String findBestDirectionWithAvoidance(
    Point current,
    Point target,
    LocalMap map,
    int agentSize,
    String blockDirection,
    Set<Point> avoidPoints
  ) {
    String bestDir = null;
    double bestScore = Double.NEGATIVE_INFINITY;

    for (String dir : DIRECTIONS) {
      if (
        !isValidMoveWithConstraints(
          current,
          dir,
          map,
          agentSize,
          blockDirection,
          false
        )
      ) {
        continue;
      }

      Point next = getNextPoint(current, dir);
      if (avoidPoints.contains(next)) continue;

      double score = scorePosition(next, target, map);
      if (score > bestScore) {
        bestScore = score;
        bestDir = dir;
      }
    }

    return bestDir;
  }

  private boolean isValidMoveWithConstraints(
    Point current,
    String direction,
    LocalMap map,
    int agentSize,
    String blockDirection,
    boolean requireMoreClearance
  ) {
    try {
      Point next = getNextPoint(current, direction);

      if (map.isForbidden(next) || map.hasObstacle(next)) {
        return false;
      }

      if (requireMoreClearance) {
        int clearDirections = countClearDirections(next, map);
        return clearDirections >= 2; // Require at least 2 clear directions
      }

      return true;
    } catch (Exception e) {
      logger.warning("Error in isValidMoveWithConstraints: " + e.getMessage());
      return false;
    }
  }

  // Helper method for string formatting in logger
  private String formatLog(String format, Object... args) {
    return String.format(format, args);
  }
}
