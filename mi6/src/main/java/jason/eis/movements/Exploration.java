package jason.eis.movements;

import jason.eis.LocalMap;
import jason.eis.Point;
import java.util.*;
import java.util.concurrent.ConcurrentHashMap;
import java.util.logging.Level;
import java.util.logging.Logger;

public class Exploration {
  private static final Logger logger = Logger.getLogger(
    Exploration.class.getName()
  );
  private static final boolean DEBUG = true; // Toggle debug logging
  private static final int ZONE_SIZE = 5;
  private static final double UNEXPLORED_BONUS = 2.0;
  private static final double REVISIT_PENALTY = 0.3;
  private static final int DECAY_TIME = 30000; // 30 seconds
  private static final double ADJACENT_ZONE_WEIGHT = 0.3;

  // Exploration constants
  private static final double DISTANCE_BONUS_FACTOR = 0.15;
  private static final int MAX_LOCAL_VISITS = 3;
  private static final double LOCAL_AREA_RADIUS = 8;
  private static final int MAX_PATH_LENGTH = 15;
  private static final int MIN_PATH_LENGTH = 5;
  private static final double PATH_SCORE_THRESHOLD = 0.7;

  // Constants for scoring weights
  private static final double EXPLORATION_WEIGHT = 0.4;
  private static final double BOUNDARY_WEIGHT = 0.2;

  // Track active exploration paths
  private final Map<String, PathResult> activePaths = new ConcurrentHashMap<>();
  private final ExplorationSearch explorationSearch = new ExplorationSearch();

  // Add at the top of the class after other constants
  private final Map<Point, Double> heatMap = new ConcurrentHashMap<>();
  private static final double STUCK_PENALTY = 2.0;
  private static final double OSCILLATION_PENALTY = 1.5;

  public enum RecomputeReason {
    PATH_BLOCKED,
    STUCK,
    OSCILLATION,
    EXPLORATION_TIMEOUT,
    BOUNDARY_CONSTRAINT,
    OBSTACLE_CONSTRAINT,
  }

  public static class PathResult {
    public final Point startPosition;
    public final List<String> directions;
    public final Point targetPosition;
    public final boolean success;
    public final RecomputeReason reason;

    public PathResult(
      Point start,
      List<String> dirs,
      Point target,
      String ignored,
      RecomputeReason reason
    ) {
      this.startPosition = start;
      this.directions = new ArrayList<>(dirs);
      this.targetPosition = target;
      this.success = true;
      this.reason = reason;
    }

    public PathResult(Point start, List<String> dirs, Point target) {
      this.startPosition = start;
      this.directions = new ArrayList<>(dirs);
      this.targetPosition = target;
      this.success = true;
      this.reason = null;
    }

    public PathResult(boolean success) {
      this.startPosition = null;
      this.directions = new ArrayList<>();
      this.targetPosition = null;
      this.success = success;
      this.reason = null;
    }
  }

  // Add debug logging helper
  private void debug(String message, Object... args) {
    if (DEBUG) {
      logger.fine(String.format(message, args));
    }
  }

  public String getNextDirection(
    String agName,
    Point currentPos,
    List<String> availableDirections,
    LocalMap map
  ) {
    try {
      if (currentPos == null) {
        logger.warning(String.format("[%s] Current position is null", agName));
        return null;
      }

      if (availableDirections == null || availableDirections.isEmpty()) {
        logger.warning(
          String.format(
            "[%s] No available directions at position %s",
            agName,
            currentPos
          )
        );
        return null;
      }

      debug(
        "[%s] Getting next direction at pos %s with available directions: %s",
        agName,
        currentPos,
        String.join(",", availableDirections)
      );

      PathResult currentPath = getActivePath(agName);
      debug(
        "[%s] Current path: %s",
        agName,
        currentPath != null
          ? String.format(
            "directions=%s, target=%s",
            currentPath.directions,
            currentPath.targetPosition
          )
          : "null"
      );

      if (currentPath != null && !currentPath.directions.isEmpty()) {
        String nextInLine = currentPath.directions.get(0);
        debug("[%s] Next direction in line: %s", agName, nextInLine);

        if (!availableDirections.contains(nextInLine)) {
          debug(
            "[%s] Next direction %s not in available directions: %s",
            agName,
            nextInLine,
            String.join(",", availableDirections)
          );
          return computeNewPath(agName, currentPos, availableDirections, map);
        }

        if (!isPathStillValid(currentPath, currentPos, map)) {
          debug("[%s] Current path is no longer valid", agName);
          return computeNewPath(agName, currentPos, availableDirections, map);
        }

        currentPath.directions.remove(0);
        debug("[%s] Using next direction from path: %s", agName, nextInLine);
        return nextInLine;
      }

      debug("[%s] No current path, computing new path", agName);
      return computeNewPath(agName, currentPos, availableDirections, map);
    } catch (Exception e) {
      logger.log(
        Level.SEVERE,
        String.format(
          "[%s] Critical error in getNextDirection: %s",
          agName,
          e.getMessage()
        ),
        e
      );
      // In case of error, try to return a safe direction
      if (availableDirections != null && !availableDirections.isEmpty()) {
        String safeDirection = selectRandomDirection(availableDirections);
        debug(
          "[%s] Returning safe direction after error: %s",
          agName,
          safeDirection
        );
        return safeDirection;
      }
      return null;
    }
  }

  private String computeNewPath(
    String agName,
    Point currentPos,
    List<String> availableDirections,
    LocalMap map
  ) {
    try {
      debug("[%s] Computing new path from position %s", agName, currentPos);

      if (availableDirections == null || availableDirections.isEmpty()) {
        logger.warning(String.format("[%s] No available directions", agName));
        return null;
      }

      // Try primary path computation first
      String bestDirection = findBestDirection(
        currentPos,
        availableDirections,
        map
      );

      // If primary method fails, use fallback heat-based selection
      if (bestDirection == null) {
        debug(
          "[%s] Primary path computation failed, using heat-based fallback",
          agName
        );
        bestDirection =
          findDirectionWithLeastHeat(
            agName,
            currentPos,
            availableDirections,
            map
          );
      }

      // If we have a direction, set it as active path
      if (bestDirection != null) {
        Point nextPosition = calculateNextPosition(currentPos, bestDirection);
        setActivePath(
          agName,
          new PathResult(
            currentPos,
            Collections.singletonList(bestDirection),
            nextPosition
          )
        );
        debug("[%s] Selected direction: %s", agName, bestDirection);
        return bestDirection;
      }

      // Final fallback: random direction with heat consideration
      debug("[%s] Using random direction fallback", agName);
      return selectRandomDirection(availableDirections);
    } catch (Exception e) {
      logger.log(
        Level.WARNING,
        String.format(
          "[%s] Error in computeNewPath: %s",
          agName,
          e.getMessage()
        ),
        e
      );
      return selectRandomDirection(availableDirections); // Ultimate fallback
    }
  }

  private String findBestDirection(
    Point currentPos,
    List<String> availableDirections,
    LocalMap map
  ) {
    String bestDirection = null;
    double bestScore = Double.NEGATIVE_INFINITY;

    for (String direction : availableDirections) {
      Point nextPos = calculateNextPosition(currentPos, direction);
      if (map.isOutOfBounds(nextPos) || map.hasObstacle(nextPos)) continue;

      double score = evaluateDirection(nextPos, map);
      if (score > bestScore) {
        bestScore = score;
        bestDirection = direction;
      }
    }

    return bestDirection;
  }

  private String findDirectionWithLeastHeat(
    String agName,
    Point currentPos,
    List<String> availableDirections,
    LocalMap map
  ) {
    String bestDirection = null;
    double lowestHeat = Double.POSITIVE_INFINITY;
    Map<Point, Double> visitHeatmap = map.getVisitedHeatmap();

    for (String direction : availableDirections) {
      Point nextPos = calculateNextPosition(currentPos, direction);
      if (map.isOutOfBounds(nextPos) || map.hasObstacle(nextPos)) continue;

      // Get heat value and add small random factor to break ties
      double heat = visitHeatmap.getOrDefault(nextPos, 0.0);
      heat += Math.random() * 0.1; // Small random factor

      // Consider unexplored areas highly favorable
      if (!map.isExplored(nextPos)) {
        heat *= 0.5; // Reduce heat for unexplored areas
      }

      // Check if this is better than our current best
      if (heat < lowestHeat) {
        lowestHeat = heat;
        bestDirection = direction;
      }
    }

    return bestDirection;
  }

  private String selectRandomDirection(List<String> availableDirections) {
    try {
      if (availableDirections == null || availableDirections.isEmpty()) {
        logger.severe("No available directions for random selection");
        return null;
      }

      List<String> shuffled = new ArrayList<>(availableDirections);
      Collections.shuffle(shuffled);
      String selected = shuffled.get(0);
      debug(
        "Randomly selected direction: %s from available: %s",
        selected,
        String.join(",", availableDirections)
      );
      return selected;
    } catch (Exception e) {
      logger.severe("Error in random direction selection: " + e.getMessage());
      return null;
    }
  }

  private double evaluateDirection(Point nextPos, LocalMap map) {
    double score = 0.0;

    // Exploration potential (higher is better)
    score += calculateUnexploredScore(nextPos, map) * EXPLORATION_WEIGHT;

    // Use LocalMap's visit tracking instead of heat map
    Map<Point, Double> visitHeatmap = map.getVisitedHeatmap();
    double visitScore = 1.0 - visitHeatmap.getOrDefault(nextPos, 0.0);
    score += visitScore * EXPLORATION_WEIGHT;

    // Boundary distance (higher is better)
    score += calculateBoundaryDistance(nextPos, map) * BOUNDARY_WEIGHT;

    return score;
  }

  public void triggerPathRecompute(
    String agName,
    Point startPoint,
    LocalMap map,
    RecomputeReason reason,
    String triggerDirection
  ) {
    try {
      debug(
        "[%s] Triggering path recompute. Reason: %s, Direction: %s",
        agName,
        reason,
        triggerDirection
      );

      PathResult newPath = findNewPath(
        agName,
        startPoint,
        map,
        reason,
        triggerDirection
      );
      if (newPath.success) {
        activePaths.put(agName, newPath);
        debug("[%s] New path computed successfully", agName);
      } else {
        activePaths.remove(agName);
        debug("[%s] Failed to compute new path", agName);
      }
    } catch (Exception e) {
      logger.log(
        Level.WARNING,
        String.format(
          "[%s] Error in triggerPathRecompute: %s",
          agName,
          e.getMessage()
        ),
        e
      );
      activePaths.remove(agName);
    }
  }

  private void setActivePath(String agName, PathResult path) {
    activePaths.put(agName, path);
  }

  private PathResult getActivePath(String agName) {
    return activePaths.get(agName);
  }

  private Point calculateNextPosition(Point current, String direction) {
    if (current == null) {
      throw new IllegalArgumentException("Current position cannot be null");
    }
    if (!isValidDirection(direction)) {
      throw new IllegalArgumentException("Invalid direction: " + direction);
    }

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
        throw new IllegalStateException("Unexpected direction: " + direction);
    }
  }

  private double calculateUnexploredScore(Point pos, LocalMap map) {
    int unexploredCount = 0;
    int totalNeighbors = 0;

    // Check surrounding cells
    for (int dx = -1; dx <= 1; dx++) {
      for (int dy = -1; dy <= 1; dy++) {
        if (dx == 0 && dy == 0) continue;
        Point neighbor = new Point(pos.x + dx, pos.y + dy);
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
    // Implementation needed
    return 0.0; // Placeholder return, actual implementation needed
  }

  private boolean isPathStillValid(
    PathResult path,
    Point currentPos,
    LocalMap map
  ) {
    try {
      if (path == null || path.targetPosition == null || currentPos == null) {
        debug("Path validation failed: null check failed");
        return false;
      }

      // Check if target is still accessible
      if (
        map.hasObstacle(path.targetPosition) ||
        map.isOutOfBounds(path.targetPosition)
      ) {
        debug(
          "Path validation failed: target position is blocked or out of bounds"
        );
        return false;
      }

      // Check if we're still on the expected path
      if (
        !isPositionNearPath(currentPos, path.startPosition, path.targetPosition)
      ) {
        debug(
          "Path validation failed: current position is too far from planned path"
        );
        return false;
      }

      return true;
    } catch (Exception e) {
      logger.warning("Error in path validation: " + e.getMessage());
      return false;
    }
  }

  private boolean isPositionNearPath(Point current, Point start, Point target) {
    // Calculate maximum allowed deviation from path
    int pathLength =
      Math.abs(target.x - start.x) + Math.abs(target.y - start.y);
    int maxDeviation = Math.max(1, pathLength / 3); // Allow some deviation based on path length

    // Calculate actual deviation
    int deviation =
      Math.abs(current.x - start.x) + Math.abs(current.y - start.y);

    return deviation <= maxDeviation;
  }

  private Point findNearestTarget(
    LocalMap map,
    Point currentPos,
    String targetType
  ) {
    // Implementation needed
    return null; // Placeholder return, actual implementation needed
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
    RecomputeReason reason,
    String triggerDirection
  ) {
    try {
      debug(
        "[%s] Finding new path. Reason: %s, Trigger Direction: %s",
        agName,
        reason,
        triggerDirection
      );

      if (current == null || map == null) {
        logger.warning(
          String.format("[%s] Invalid parameters for findNewPath", agName)
        );
        return new PathResult(false);
      }

      switch (reason) {
        case STUCK:
          return handleStuckCase(agName, current, map, triggerDirection);
        case OSCILLATION:
          return handleOscillationCase(agName, current, map, triggerDirection);
        default:
          return handleDefaultRecompute(agName, current, map, triggerDirection);
      }
    } catch (Exception e) {
      logger.log(
        Level.SEVERE,
        String.format(
          "[%s] Error finding new path: %s",
          agName,
          e.getMessage()
        ),
        e
      );
      return new PathResult(false);
    }
  }

  private PathResult handleStuckCase(
    String agName,
    Point current,
    LocalMap map,
    String stuckDirection
  ) {
    try {
      debug(
        "[%s] Handling stuck case. Stuck direction: %s",
        agName,
        stuckDirection
      );

      // Use the trigger direction as the stuck direction
      String oppositeDir = getOppositeDirection(stuckDirection);
      debug("[%s] Attempting opposite direction: %s", agName, oppositeDir);

      // Add very high heat to stuck area
      addStuckPenalty(current);

      // Get available directions excluding the stuck direction
      List<String> availableDirections = getAvailableDirections(current, map);
      availableDirections.remove(stuckDirection);

      // Prioritize opposite direction if available
      if (oppositeDir != null && availableDirections.contains(oppositeDir)) {
        Point nextPos = calculateNextPosition(current, oppositeDir);
        debug("[%s] Using opposite direction for escape", agName);
        return new PathResult(
          current,
          Collections.singletonList(oppositeDir),
          nextPos,
          null,
          RecomputeReason.STUCK
        );
      }

      debug(
        "[%s] Opposite direction not available, finding alternative escape path",
        agName
      );
      return findEscapePath(current, map, availableDirections);
    } catch (Exception e) {
      logger.log(
        Level.WARNING,
        String.format(
          "[%s] Error handling stuck case: %s",
          agName,
          e.getMessage()
        ),
        e
      );
      return new PathResult(false);
    }
  }

  private PathResult handleOscillationCase(
    String agName,
    Point current,
    LocalMap map,
    String oscillatingDirection
  ) {
    try {
      debug(
        "[%s] Handling oscillation case. Oscillating direction: %s",
        agName,
        oscillatingDirection
      );

      Set<String> oscillatingDirections = new HashSet<>();
      oscillatingDirections.add(oscillatingDirection);
      oscillatingDirections.add(getOppositeDirection(oscillatingDirection));

      // Add very high heat to oscillation area
      addOscillationPenalty(current);

      // Get available directions excluding oscillating ones
      List<String> availableDirections = getAvailableDirections(current, map);
      availableDirections.removeAll(oscillatingDirections);

      if (availableDirections.isEmpty()) {
        // If no alternative directions, force a perpendicular path
        List<String> perpendicularDirs = getPerpendicularDirections(
          oscillatingDirection
        );
        for (String dir : perpendicularDirs) {
          Point nextPos = calculateNextPosition(current, dir);
          if (!map.hasObstacle(nextPos) && !map.isOutOfBounds(nextPos)) {
            debug("[%s] Using perpendicular direction: %s", agName, dir);
            return new PathResult(
              current,
              Collections.singletonList(dir),
              nextPos,
              null,
              RecomputeReason.OSCILLATION
            );
          }
        }
        return new PathResult(false);
      }

      debug("[%s] Finding escape path avoiding oscillation directions", agName);
      return findEscapePath(current, map, availableDirections);
    } catch (Exception e) {
      logger.log(
        Level.WARNING,
        String.format(
          "[%s] Error handling oscillation case: %s",
          agName,
          e.getMessage()
        ),
        e
      );
      return new PathResult(false);
    }
  }

  private PathResult handleDefaultRecompute(
    String agName,
    Point current,
    LocalMap map,
    String triggerDirection
  ) {
    try {
      debug(
        "[%s] Handling default recompute. Trigger direction: %s",
        agName,
        triggerDirection
      );

      // Get available directions excluding the trigger direction
      List<String> availableDirections = getAvailableDirections(current, map);
      availableDirections.remove(triggerDirection);

      if (availableDirections.isEmpty()) {
        debug("[%s] No available directions for default recompute", agName);
        return new PathResult(false);
      }

      return findEscapePath(current, map, availableDirections);
    } catch (Exception e) {
      logger.log(
        Level.WARNING,
        String.format(
          "[%s] Error handling default recompute: %s",
          agName,
          e.getMessage()
        ),
        e
      );
      return new PathResult(false);
    }
  }

  private PathResult findEscapePath(
    Point current,
    LocalMap map,
    List<String> availableDirections
  ) {
    if (availableDirections.isEmpty()) {
      return new PathResult(false);
    }

    String bestDirection = null;
    double bestScore = Double.NEGATIVE_INFINITY;

    for (String direction : availableDirections) {
      Point nextPos = calculateNextPosition(current, direction);

      // Calculate score based on:
      // 1. Distance from current position (further is better)
      // 2. Heat map value (lower is better)
      // 3. Number of available moves from next position
      double distanceScore = 1.0; // Base score
      double heatScore = 1.0 - heatMap.getOrDefault(nextPos, 0.0);
      int availableMoves = countAvailableMoves(nextPos, map);

      double score =
        (distanceScore * 0.3) + (heatScore * 0.4) + (availableMoves * 0.3);

      if (score > bestScore) {
        bestScore = score;
        bestDirection = direction;
      }
    }

    if (bestDirection != null) {
      Point nextPos = calculateNextPosition(current, bestDirection);
      return new PathResult(
        current,
        Collections.singletonList(bestDirection),
        nextPos,
        null,
        RecomputeReason.EXPLORATION_TIMEOUT
      );
    }

    return new PathResult(false);
  }

  private int countAvailableMoves(Point pos, LocalMap map) {
    int count = 0;
    String[] directions = { "n", "s", "e", "w" };
    for (String dir : directions) {
      Point nextPos = calculateNextPosition(pos, dir);
      if (!map.hasObstacle(nextPos) && !map.isOutOfBounds(nextPos)) {
        count++;
      }
    }
    return count;
  }

  private List<String> getPerpendicularDirections(String direction) {
    List<String> perpendicular = new ArrayList<>();
    if (direction == null) return perpendicular;

    switch (direction) {
      case "n":
      case "s":
        perpendicular.add("e");
        perpendicular.add("w");
        break;
      case "e":
      case "w":
        perpendicular.add("n");
        perpendicular.add("s");
        break;
    }
    return perpendicular;
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

    private static class SearchNode implements Comparable<SearchNode> {
      Point position;
      SearchNode parent;
      String direction;
      double score;
      int depth;

      SearchNode(
        Point pos,
        SearchNode parent,
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
      public int compareTo(SearchNode other) {
        return Double.compare(other.score, this.score); // Higher score first
      }
    }

    public PathResult findExplorationPath(
      Point start,
      LocalMap map,
      Map<Point, Double> heatMap
    ) {
      PriorityQueue<SearchNode> frontier = new PriorityQueue<>();
      Set<Point> visited = new HashSet<>();
      SearchNode bestNode = null;
      double bestScore = Double.NEGATIVE_INFINITY;

      frontier.add(new SearchNode(start, null, null, 0, 0));

      while (!frontier.isEmpty()) {
        SearchNode current = frontier.poll();

        if (visited.contains(current.position)) continue;
        visited.add(current.position);

        double currentScore = evaluateExplorationScore(current, map, heatMap);
        if (currentScore > bestScore) {
          bestScore = currentScore;
          bestNode = current;
        }

        if (current.depth >= MAX_DEPTH) continue;

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
              new SearchNode(
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
      int depth
    ) {
      int unexploredCount = countUnexploredNeighbors(pos, map);
      double unexploredScore = unexploredCount / 4.0;
      double heatPenalty = heatMap.getOrDefault(pos, 0.0);
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
        if (!map.isOutOfBounds(neighbor) && !map.hasObstacle(neighbor)) {
          count++;
        }
      }
      return count;
    }

    private double evaluateExplorationScore(
      SearchNode node,
      LocalMap map,
      Map<Point, Double> heatMap
    ) {
      return calculateDirectionScore(node.position, map, heatMap, node.depth);
    }

    private boolean isValidMove(Point pos, LocalMap map) {
      return !map.hasObstacle(pos) && !map.isOutOfBounds(pos);
    }

    private Point getNextPosition(Point current, String direction) {
      Point vector = DIRECTION_VECTORS.get(direction);
      return new Point(current.x + vector.x, current.y + vector.y);
    }

    private PathResult reconstructPath(SearchNode node) {
      List<String> directions = new ArrayList<>();
      Point startPos = null;

      SearchNode current = node;
      while (current.parent != null) {
        directions.add(0, current.direction);
        if (startPos == null) {
          startPos = current.parent.position;
        }
        current = current.parent;
      }

      return new PathResult(startPos, directions, node.position);
    }
  }

  // Add validation helper method
  private boolean isValidDirection(String direction) {
    return (
      direction != null && Arrays.asList("n", "s", "e", "w").contains(direction)
    );
  }

  // Add heat map management methods
  private void addStuckPenalty(Point position) {
    try {
      double currentHeat = heatMap.getOrDefault(position, 0.0);
      heatMap.put(position, currentHeat + STUCK_PENALTY);
      debug(
        "Added stuck penalty to position %s. New heat: %f",
        position,
        currentHeat + STUCK_PENALTY
      );
    } catch (Exception e) {
      logger.warning("Error adding stuck penalty: " + e.getMessage());
    }
  }

  private void addOscillationPenalty(Point position) {
    try {
      double currentHeat = heatMap.getOrDefault(position, 0.0);
      heatMap.put(position, currentHeat + OSCILLATION_PENALTY);
      debug(
        "Added oscillation penalty to position %s. New heat: %f",
        position,
        currentHeat + OSCILLATION_PENALTY
      );
    } catch (Exception e) {
      logger.warning("Error adding oscillation penalty: " + e.getMessage());
    }
  }

  // Add direction validation method
  private List<String> getAvailableDirections(Point current, LocalMap map) {
    try {
      List<String> directions = new ArrayList<>();
      String[] possibleDirs = { "n", "s", "e", "w" };

      for (String dir : possibleDirs) {
        Point nextPos = calculateNextPosition(current, dir);
        if (!map.hasObstacle(nextPos) && !map.isOutOfBounds(nextPos)) {
          directions.add(dir);
        }
      }

      return directions;
    } catch (Exception e) {
      logger.warning("Error getting available directions: " + e.getMessage());
      return new ArrayList<>();
    }
  }

  // Update triggerPathRecompute method signature in RandomMovement class
  public void triggerPathRecompute(
    String agName,
    Point startPoint,
    LocalMap map,
    RecomputeReason reason
  ) {
    triggerPathRecompute(agName, startPoint, map, reason, null);
  }
}
