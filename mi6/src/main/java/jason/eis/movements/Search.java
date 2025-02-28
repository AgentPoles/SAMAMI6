package jason.eis.movements;

import jason.eis.LocalMap;
import jason.eis.Point;
import java.util.*;

public class Search {

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
    DIRECTION_VECTORS.put("n", new Point(0, -1));
    DIRECTION_VECTORS.put("e", new Point(1, 0));
    DIRECTION_VECTORS.put("s", new Point(0, 1));
    DIRECTION_VECTORS.put("w", new Point(-1, 0));
  }

  private static class JumpPoint {
    Point position;
    JumpPoint parent;
    double gCost; // Cost from start
    double hCost; // Estimated cost to goal
    String direction; // Direction taken to reach this point

    JumpPoint(Point pos, JumpPoint parent, double g, double h, String dir) {
      this.position = pos;
      this.parent = parent;
      this.gCost = g;
      this.hCost = h;
      this.direction = dir;
    }

    double fCost() {
      return gCost + hCost;
    }
  }

  public static class PathResult {
    public final List<String> directions;
    public final List<Point> points;
    public final boolean success;

    PathResult(List<String> dirs, List<Point> pts, boolean success) {
      this.directions = dirs;
      this.points = pts;
      this.success = success;
    }
  }

  public PathResult findPath(
    Point start,
    Point target,
    LocalMap map,
    TargetType targetType
  ) {
    PriorityQueue<JumpPoint> openSet = new PriorityQueue<>(
      Comparator.comparingDouble(JumpPoint::fCost)
    );
    Set<Point> closedSet = new HashSet<>();

    JumpPoint startPoint = new JumpPoint(
      start,
      null,
      0,
      heuristic(start, target),
      null
    );
    openSet.add(startPoint);

    while (!openSet.isEmpty()) {
      JumpPoint current = openSet.poll();

      if (isAtTarget(current.position, target, targetType)) {
        return reconstructPath(current);
      }

      closedSet.add(current.position);

      // Identify successors using jumping rules
      for (String direction : DIRECTIONS) {
        Point jumpPoint = jump(current.position, direction, map, target);
        if (jumpPoint == null || closedSet.contains(jumpPoint)) continue;

        double newG =
          current.gCost + euclideanDistance(current.position, jumpPoint);
        JumpPoint successor = new JumpPoint(
          jumpPoint,
          current,
          newG,
          heuristic(jumpPoint, target),
          direction
        );

        if (!openSet.contains(successor)) {
          openSet.add(successor);
        }
      }
    }

    return new PathResult(new ArrayList<>(), new ArrayList<>(), false);
  }

  private Point jump(
    Point current,
    String direction,
    LocalMap map,
    Point target
  ) {
    Point next = applyDirection(current, direction);

    if (!isWalkable(next, map)) return null;
    if (isAtTarget(next, target, null)) return next; // Found target

    // Check for forced neighbors
    if (hasForceNeighbor(next, direction, map)) {
      return next;
    }

    // Diagonal movement (will implement later for size > 1)

    // Continue jumping
    return jump(next, direction, map, target);
  }

  private boolean hasForceNeighbor(Point pos, String direction, LocalMap map) {
    // Check for obstacles that create forced neighbors
    Point[] neighbors = getNeighbors(pos, direction);
    for (Point neighbor : neighbors) {
      if (!isWalkable(neighbor, map)) {
        return true;
      }
    }
    return false;
  }

  private Point[] getNeighbors(Point pos, String direction) {
    // Return relevant neighboring points based on direction
    switch (direction) {
      case "n":
      case "s":
        return new Point[] {
          new Point(pos.x - 1, pos.y),
          new Point(pos.x + 1, pos.y),
        };
      case "e":
      case "w":
        return new Point[] {
          new Point(pos.x, pos.y - 1),
          new Point(pos.x, pos.y + 1),
        };
      default:
        return new Point[0];
    }
  }

  private boolean isWalkable(Point pos, LocalMap map) {
    return !map.hasObstacle(pos) && !map.isOutOfBounds(pos);
  }

  private boolean isAtTarget(Point pos, Point target, TargetType type) {
    // For size 1, simple position check
    return pos.equals(target);
  }

  private double heuristic(Point a, Point b) {
    // Manhattan distance for grid movement
    return Math.abs(a.x - b.x) + Math.abs(a.y - b.y);
  }

  private double euclideanDistance(Point a, Point b) {
    double dx = a.x - b.x;
    double dy = a.y - b.y;
    return Math.sqrt(dx * dx + dy * dy);
  }

  private Point applyDirection(Point pos, String direction) {
    Point vector = DIRECTION_VECTORS.get(direction);
    return new Point(pos.x + vector.x, pos.y + vector.y);
  }

  private PathResult reconstructPath(JumpPoint end) {
    List<String> directions = new ArrayList<>();
    List<Point> points = new ArrayList<>();

    JumpPoint current = end;
    while (current.parent != null) {
      directions.add(0, current.direction);
      points.add(0, current.position);
      current = current.parent;
    }
    points.add(0, current.position); // Add start position

    return new PathResult(directions, points, true);
  }
}
