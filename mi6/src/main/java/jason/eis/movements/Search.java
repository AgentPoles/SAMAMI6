package jason.eis.movements;

import jason.eis.LocalMap;
import jason.eis.Point;
import java.util.*;
import java.util.logging.Logger;

public class Search {
  private static final Logger logger = Logger.getLogger(Search.class.getName());
  private static final boolean DEBUG = true;
  private static final int MAX_ITERATIONS = 10000; // Prevent infinite loops
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
      this.directions = Collections.unmodifiableList(new ArrayList<>(dirs));
      this.points = Collections.unmodifiableList(new ArrayList<>(pts));
      this.success = success;
    }
  }

  public PathResult findPath(
    Point start,
    Point target,
    LocalMap map,
    TargetType targetType
  ) {
    try {
      // Validate input parameters
      if (!validateInputs(start, target, map, targetType)) {
        return createEmptyPathResult("Invalid input parameters");
      }

      // Priority queue ordered by f-cost (g + h)
      PriorityQueue<JumpPoint> openSet = new PriorityQueue<>(
        Comparator.comparingDouble(JumpPoint::fCost)
      );

      // Track visited nodes and their best g-costs
      Map<Point, Double> gScores = new HashMap<>();
      Set<Point> closedSet = new HashSet<>();

      // Initialize start point
      JumpPoint startPoint = createStartPoint(start, target);
      openSet.add(startPoint);
      gScores.put(start, 0.0);

      int iterations = 0;
      while (!openSet.isEmpty() && iterations < MAX_ITERATIONS) {
        iterations++;
        JumpPoint current = openSet.poll();

        if (current == null) {
          return createEmptyPathResult("Null current point in pathfinding");
        }

        // Found target
        if (isAtTarget(current.position, target, targetType)) {
          return reconstructPath(current);
        }

        closedSet.add(current.position);

        try {
          processNeighbors(current, target, map, openSet, gScores, closedSet);
        } catch (Exception e) {
          logger.warning("Error processing neighbors: " + e.getMessage());
          continue;
        }
      }

      if (iterations >= MAX_ITERATIONS) {
        logger.warning("Pathfinding exceeded maximum iterations");
      }

      return createEmptyPathResult("No path found");
    } catch (Exception e) {
      logger.severe("Critical error in pathfinding: " + e.getMessage());
      return createEmptyPathResult("Pathfinding failed");
    }
  }

  private boolean validateInputs(
    Point start,
    Point target,
    LocalMap map,
    TargetType targetType
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

    if (!isWalkable(start, map)) {
      logger.warning("Start position is not walkable: " + start);
      return false;
    }

    return true;
  }

  private void processNeighbors(
    JumpPoint current,
    Point target,
    LocalMap map,
    PriorityQueue<JumpPoint> openSet,
    Map<Point, Double> gScores,
    Set<Point> closedSet
  ) {
    for (String direction : DIRECTIONS) {
      try {
        Point nextPos = applyDirection(current.position, direction);
        if (
          nextPos == null ||
          !isWalkable(nextPos, map) ||
          closedSet.contains(nextPos)
        ) {
          continue;
        }

        double newG = current.gCost + 1.0;
        if (newG > MAX_PATH_COST) {
          continue; // Path too long, skip
        }

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
      } catch (Exception e) {
        logger.warning(
          "Error processing direction " + direction + ": " + e.getMessage()
        );
      }
    }
  }

  private boolean isWalkable(Point pos, LocalMap map) {
    try {
      return !map.hasObstacle(pos) && !map.isOutOfBounds(pos);
    } catch (Exception e) {
      logger.warning(
        "Error checking walkable position " + pos + ": " + e.getMessage()
      );
      return false;
    }
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

  private PathResult reconstructPath(JumpPoint end) {
    try {
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
    } catch (Exception e) {
      logger.severe("Error reconstructing path: " + e.getMessage());
      return createEmptyPathResult("Path reconstruction failed");
    }
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
}
