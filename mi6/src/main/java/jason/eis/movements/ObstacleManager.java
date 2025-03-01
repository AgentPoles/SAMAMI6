package jason.eis.movements;

import jason.eis.LocalMap;
import jason.eis.Point;
import java.util.*;

public class ObstacleManager {
  private static final double BLOCK_BUFFER = 2.0; // Extra space around blocks
  private static final int CRITICAL_DISTANCE = 1;

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
    return availableDirections
      .stream()
      .filter(dir -> isValidMove(dir, agentPos, agentSize, blockDirection, map))
      .toList();
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
    // Get all positions the agent will occupy after move
    Set<Point> nextAgentPositions = getAgentPositions(
      calculateNextPosition(agentPos, moveDirection),
      agentSize
    );

    // Check agent positions
    if (
      nextAgentPositions.stream().anyMatch(pos -> isPositionInvalid(pos, map))
    ) {
      return false;
    }

    // If has block, check block positions too
    if (blockDirection != null) {
      Set<Point> nextBlockPositions = getBlockPositions(
        nextAgentPositions,
        blockDirection,
        agentSize
      );
      if (
        nextBlockPositions.stream().anyMatch(pos -> isPositionInvalid(pos, map))
      ) {
        return false;
      }

      // Check path between current and next positions
      Set<Point> currentAgentPositions = getAgentPositions(agentPos, agentSize);
      Set<Point> currentBlockPositions = getBlockPositions(
        currentAgentPositions,
        blockDirection,
        agentSize
      );

      if (
        hasObstacleInPath(
          currentAgentPositions,
          nextAgentPositions,
          currentBlockPositions,
          nextBlockPositions,
          map
        )
      ) {
        return false;
      }
    }

    return true;
  }

  /**
   * Check if position is invalid (obstacle or out of bounds)
   */
  private boolean isPositionInvalid(Point pos, LocalMap map) {
    return map.hasObstacle(pos) || map.isOutOfBounds(pos);
  }

  /**
   * Get all positions occupied by agent of given size
   */
  private Set<Point> getAgentPositions(Point topLeft, int size) {
    Set<Point> positions = new HashSet<>();
    for (int dx = 0; dx < size; dx++) {
      for (int dy = 0; dy < size; dy++) {
        positions.add(new Point(topLeft.x + dx, topLeft.y + dy));
      }
    }
    return positions;
  }

  /**
   * Get all positions occupied by attached block
   */
  private Set<Point> getBlockPositions(
    Set<Point> agentPositions,
    String blockDirection,
    int agentSize
  ) {
    Set<Point> blockPositions = new HashSet<>();
    for (Point agentPos : agentPositions) {
      Point blockPos = getBlockPosition(agentPos, blockDirection);
      // Only add if this block position isn't already occupied by agent
      if (!agentPositions.contains(blockPos)) {
        blockPositions.add(blockPos);
      }
    }
    return blockPositions;
  }

  /**
   * Check for obstacles in movement path considering all positions
   */
  private boolean hasObstacleInPath(
    Set<Point> currentAgent,
    Set<Point> nextAgent,
    Set<Point> currentBlock,
    Set<Point> nextBlock,
    LocalMap map
  ) {
    Set<Point> pathPoints = new HashSet<>();
    pathPoints.addAll(currentAgent);
    pathPoints.addAll(nextAgent);
    pathPoints.addAll(currentBlock);
    pathPoints.addAll(nextBlock);

    // Add intermediate points for non-adjacent movements
    for (Point from : currentAgent) {
      for (Point to : nextAgent) {
        addIntermediatePoints(pathPoints, from, to);
      }
    }

    for (Point from : currentBlock) {
      for (Point to : nextBlock) {
        addIntermediatePoints(pathPoints, from, to);
      }
    }

    return pathPoints
      .stream()
      .anyMatch(point -> map.hasObstacle(point) || map.isOutOfBounds(point));
  }

  /**
   * Calculate next position given current position and direction
   */
  private Point calculateNextPosition(Point current, String direction) {
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

  /**
   * Get block position relative to agent position
   */
  private Point getBlockPosition(Point agentPos, String blockDirection) {
    switch (blockDirection) {
      case "n":
        return new Point(agentPos.x, agentPos.y - 1);
      case "s":
        return new Point(agentPos.x, agentPos.y + 1);
      case "e":
        return new Point(agentPos.x + 1, agentPos.y);
      case "w":
        return new Point(agentPos.x - 1, agentPos.y);
      default:
        return agentPos;
    }
  }

  /**
   * Add intermediate points between two positions
   */
  private void addIntermediatePoints(Set<Point> points, Point from, Point to) {
    int dx = to.x - from.x;
    int dy = to.y - from.y;

    if (Math.abs(dx) > 1 || Math.abs(dy) > 1) {
      int steps = Math.max(Math.abs(dx), Math.abs(dy));
      for (int i = 1; i < steps; i++) {
        int x = from.x + (dx * i) / steps;
        int y = from.y + (dy * i) / steps;
        points.add(new Point(x, y));
      }
    }
  }
}
