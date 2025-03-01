package jason.eis.movements;

import jason.eis.LocalMap;
import jason.eis.Point;
import java.util.*;

public class ObstacleManager {
  private static final double BLOCK_BUFFER = 2.0; // Extra space around blocks
  private static final int CRITICAL_DISTANCE = 1;

  // Cache for frequently used calculations
  private final Map<String, Point> directionOffsets;
  private final Map<Integer, Set<Point>> agentSizeCache;

  public ObstacleManager() {
    // Initialize direction offset cache
    directionOffsets = new HashMap<>();
    directionOffsets.put("n", new Point(0, -1));
    directionOffsets.put("s", new Point(0, 1));
    directionOffsets.put("e", new Point(1, 0));
    directionOffsets.put("w", new Point(-1, 0));

    // Initialize agent size cache
    agentSizeCache = new HashMap<>();
  }

  /**
   * Filter directions considering agent size and block
   */
  public List<String> filterDirections(
    List<String> availableDirections,
    LocalMap map,
    Point agentPos,
    int agentSize,
    String blockDirection
  ) {
    // Use ArrayList for better performance than stream
    List<String> validDirections = new ArrayList<>();
    for (String dir : availableDirections) {
      if (isValidMove(dir, agentPos, agentSize, blockDirection, map)) {
        validDirections.add(dir);
      }
    }
    return validDirections;
  }

  /**
   * Check if move is valid considering agent size and block
   */
  private boolean isValidMove(
    String moveDirection,
    Point agentPos,
    int agentSize,
    String blockDirection,
    LocalMap map
  ) {
    // Quick bounds check first
    Point nextPos = calculateNextPosition(agentPos, moveDirection);
    if (isPositionInvalid(nextPos, map)) {
      return false;
    }

    // Get agent positions using cached patterns
    Set<Point> nextAgentPositions = getAgentPositions(nextPos, agentSize);

    // Check agent positions (using early return)
    for (Point pos : nextAgentPositions) {
      if (isPositionInvalid(pos, map)) {
        return false;
      }
    }

    // If no block, we're done
    if (blockDirection == null) {
      return true;
    }

    // Check block positions
    Set<Point> nextBlockPositions = getBlockPositions(
      nextAgentPositions,
      blockDirection
    );
    for (Point pos : nextBlockPositions) {
      if (isPositionInvalid(pos, map)) {
        return false;
      }
    }

    // Only check path if we're moving with a block
    Set<Point> currentAgentPositions = getAgentPositions(agentPos, agentSize);
    Set<Point> currentBlockPositions = getBlockPositions(
      currentAgentPositions,
      blockDirection
    );

    return !hasObstacleInPath(
      currentAgentPositions,
      nextAgentPositions,
      currentBlockPositions,
      nextBlockPositions,
      map
    );
  }

  /**
   * Check if position is invalid (obstacle or out of bounds)
   */
  private boolean isPositionInvalid(Point pos, LocalMap map) {
    return map.hasObstacle(pos) || map.isOutOfBounds(pos);
  }

  /**
   * Get all positions occupied by agent of given size (cached)
   */
  private Set<Point> getAgentPositions(Point topLeft, int size) {
    // Check cache first
    Set<Point> basePattern = agentSizeCache.computeIfAbsent(
      size,
      k -> {
        Set<Point> pattern = new HashSet<>();
        for (int dx = 0; dx < size; dx++) {
          for (int dy = 0; dy < size; dy++) {
            pattern.add(new Point(dx, dy));
          }
        }
        return pattern;
      }
    );

    // Translate pattern to actual position
    Set<Point> positions = new HashSet<>();
    for (Point offset : basePattern) {
      positions.add(new Point(topLeft.x + offset.x, topLeft.y + offset.y));
    }
    return positions;
  }

  /**
   * Get block positions more efficiently
   */
  private Set<Point> getBlockPositions(
    Set<Point> agentPositions,
    String blockDirection
  ) {
    Point offset = directionOffsets.get(blockDirection);
    if (offset == null) {
      return Collections.emptySet();
    }

    Set<Point> blockPositions = new HashSet<>();
    for (Point agentPos : agentPositions) {
      Point blockPos = new Point(agentPos.x + offset.x, agentPos.y + offset.y);
      if (!agentPositions.contains(blockPos)) {
        blockPositions.add(blockPos);
      }
    }
    return blockPositions;
  }

  /**
   * Optimized path obstacle check
   */
  private boolean hasObstacleInPath(
    Set<Point> currentAgent,
    Set<Point> nextAgent,
    Set<Point> currentBlock,
    Set<Point> nextBlock,
    LocalMap map
  ) {
    // Use HashSet for better performance
    Set<Point> pathPoints = new HashSet<>(
      currentAgent.size() +
      nextAgent.size() +
      currentBlock.size() +
      nextBlock.size() +
      10
    ); // +10 for potential intermediate points

    pathPoints.addAll(currentAgent);
    pathPoints.addAll(nextAgent);
    pathPoints.addAll(currentBlock);
    pathPoints.addAll(nextBlock);

    // Only add intermediate points if movement is non-adjacent
    if (!areAdjacent(currentAgent, nextAgent)) {
      addIntermediatePoints(pathPoints, currentAgent, nextAgent);
    }
    if (!areAdjacent(currentBlock, nextBlock)) {
      addIntermediatePoints(pathPoints, currentBlock, nextBlock);
    }

    // Check all points in one pass
    for (Point point : pathPoints) {
      if (isPositionInvalid(point, map)) {
        return true;
      }
    }
    return false;
  }

  private boolean areAdjacent(Set<Point> set1, Set<Point> set2) {
    for (Point p1 : set1) {
      for (Point p2 : set2) {
        if (Math.abs(p1.x - p2.x) <= 1 && Math.abs(p1.y - p2.y) <= 1) {
          return true;
        }
      }
    }
    return false;
  }

  private void addIntermediatePoints(
    Set<Point> points,
    Set<Point> fromSet,
    Set<Point> toSet
  ) {
    for (Point from : fromSet) {
      for (Point to : toSet) {
        int dx = to.x - from.x;
        int dy = to.y - from.y;
        if (Math.abs(dx) > 1 || Math.abs(dy) > 1) {
          int steps = Math.max(Math.abs(dx), Math.abs(dy));
          for (int i = 1; i < steps; i++) {
            points.add(
              new Point(from.x + (dx * i) / steps, from.y + (dy * i) / steps)
            );
          }
        }
      }
    }
  }

  /**
   * Calculate next position given current position and direction
   */
  private Point calculateNextPosition(Point current, String direction) {
    Point offset = directionOffsets.get(direction);
    return offset != null
      ? new Point(current.x + offset.x, current.y + offset.y)
      : current;
  }
}
