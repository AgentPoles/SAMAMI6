package jason.eis.movements;

import jason.eis.LocalMap;
import jason.eis.Point;
import java.util.*;
import java.util.concurrent.ConcurrentHashMap;
import java.util.logging.Logger;

public class Search {
  private static final Logger logger = Logger.getLogger(Search.class.getName());
  private static final boolean DEBUG = false;
  private static final int MAX_ITERATIONS = 200;
  private static final String[] DIRECTIONS = { "n", "e", "s", "w" };
  private static final Map<String, Point> DIRECTION_VECTORS = new HashMap<>();
  private final ObstacleManager obstacleManager;

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

      if (isNearby(start, target)) {
        return findDirectPath(start, target, map, agentSize, blockDirection);
      }

      return findWaypointPath(start, target, map, agentSize, blockDirection);
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

  private PathResult findDirectPath(
    Point start,
    Point target,
    LocalMap map,
    int agentSize,
    String blockDirection
  ) {
    List<String> directions = new ArrayList<>();
    List<Point> points = new ArrayList<>();
    Point current = start;

    int iterations = 0;
    while (!current.equals(target) && iterations < MAX_ITERATIONS) {
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
            Collections.singletonList(direction),
            map,
            current,
            agentSize,
            blockDirection
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

  private PathResult findWaypointPath(
    Point start,
    Point target,
    LocalMap map,
    int agentSize,
    String blockDirection
  ) {
    List<Point> waypoints = new ArrayList<>();
    waypoints.add(start);

    Point current = start;
    while (!isNearby(current, target)) {
      Point waypoint = findNextWaypoint(
        current,
        target,
        map,
        agentSize,
        blockDirection
      );
      if (waypoint == null) break;

      waypoints.add(waypoint);
      current = waypoint;
    }
    waypoints.add(target);

    return connectWaypoints(waypoints, map, agentSize, blockDirection);
  }

  private Point findNextWaypoint(
    Point current,
    Point target,
    LocalMap map,
    int agentSize,
    String blockDirection
  ) {
    int dx = target.x - current.x;
    int dy = target.y - current.y;
    int distance = 10;

    Point[] candidates = {
      new Point(
        current.x + (int) (distance * Math.signum(dx)),
        current.y + (int) (distance * Math.signum(dy))
      ),
      new Point(current.x + (int) (distance * Math.signum(dx)), current.y),
      new Point(current.x, current.y + (int) (distance * Math.signum(dy))),
    };

    for (Point candidate : candidates) {
      if (isSafePoint(candidate, map, agentSize, blockDirection)) {
        return candidate;
      }
    }

    return null;
  }

  private boolean isSafePoint(
    Point point,
    LocalMap map,
    int agentSize,
    String blockDirection
  ) {
    return (
      obstacleManager
        .filterDirections(
          Arrays.asList(DIRECTIONS),
          map,
          point,
          agentSize,
          blockDirection
        )
        .size() >
      0
    );
  }

  private PathResult connectWaypoints(
    List<Point> waypoints,
    LocalMap map,
    int agentSize,
    String blockDirection
  ) {
    List<String> finalDirections = new ArrayList<>();
    List<Point> finalPoints = new ArrayList<>();

    for (int i = 0; i < waypoints.size() - 1; i++) {
      PathResult segment = findDirectPath(
        waypoints.get(i),
        waypoints.get(i + 1),
        map,
        agentSize,
        blockDirection
      );
      if (!segment.success) continue;

      finalPoints.addAll(segment.points);
      finalDirections.addAll(segment.directions);
    }

    boolean success = !finalPoints.isEmpty();
    return new PathResult(finalDirections, finalPoints, success);
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
      possibleDirs,
      map,
      current,
      agentSize,
      blockDirection
    );

    return validDirs.isEmpty() ? null : validDirs.get(0);
  }

  private Point getNextPoint(Point current, String direction) {
    Point vector = DIRECTION_VECTORS.get(direction);
    if (vector == null) return current;
    return new Point(current.x + vector.x, current.y + vector.y);
  }
}
