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
    // Priority queue ordered by f-cost (g + h)
    PriorityQueue<JumpPoint> openSet = new PriorityQueue<>(
      Comparator.comparingDouble(JumpPoint::fCost)
    );

    // Track visited nodes and their best g-costs
    Map<Point, Double> gScores = new HashMap<>();
    Set<Point> closedSet = new HashSet<>();

    // Initialize start point
    JumpPoint startPoint = new JumpPoint(
      start,
      null,
      0,
      heuristic(start, target),
      null
    );
    openSet.add(startPoint);
    gScores.put(start, 0.0);

    while (!openSet.isEmpty()) {
      JumpPoint current = openSet.poll();

      // Found target
      if (isAtTarget(current.position, target, targetType)) {
        return reconstructPath(current);
      }

      closedSet.add(current.position);

      // Check each possible direction
      for (String direction : DIRECTIONS) {
        Point nextPos = applyDirection(current.position, direction);

        // Skip if not walkable or already evaluated
        if (!isWalkable(nextPos, map) || closedSet.contains(nextPos)) {
          continue;
        }

        // Calculate new path cost to this neighbor
        double newG = current.gCost + 1.0; // Cost of 1 for each step

        // If we found a better path to this point
        if (!gScores.containsKey(nextPos) || newG < gScores.get(nextPos)) {
          gScores.put(nextPos, newG);
          double h = heuristic(nextPos, target);
          JumpPoint neighbor = new JumpPoint(
            nextPos,
            current,
            newG,
            h,
            direction
          );
          openSet.add(neighbor);
        }
      }
    }

    // No path found
    return new PathResult(new ArrayList<>(), new ArrayList<>(), false);
  }

  private boolean isWalkable(Point pos, LocalMap map) {
    // Check for obstacles and map boundaries
    return !map.hasObstacle(pos) && !map.isOutOfBounds(pos);
  }

  private double heuristic(Point a, Point b) {
    // Manhattan distance is admissible for 4-directional grid movement
    return Math.abs(a.x - b.x) + Math.abs(a.y - b.y);
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

  private boolean isAtTarget(Point pos, Point target, TargetType type) {
    // For size 1, simple position check
    return pos.equals(target);
  }
}
