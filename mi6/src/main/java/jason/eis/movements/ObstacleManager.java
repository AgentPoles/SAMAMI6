package jason.eis.movements;

import jason.eis.LocalMap;
import jason.eis.Point;
import java.util.*;
import java.util.logging.Logger;
import java.util.stream.Collectors;

public class ObstacleManager {
  private static final Logger logger = Logger.getLogger(
    ObstacleManager.class.getName()
  );
  private static final boolean DEBUG = true;
  private static final double BLOCK_BUFFER = 2.0; // Extra space around blocks
  private static final int CRITICAL_DISTANCE = 1;

  // Cache for frequently used calculations
  private final Map<String, Point> directionOffsets;
  private final Map<Integer, Set<Point>> agentSizeCache;

  public ObstacleManager() {
    try {
      directionOffsets = new HashMap<>();
      directionOffsets.put("n", new Point(0, -1));
      directionOffsets.put("s", new Point(0, 1));
      directionOffsets.put("e", new Point(1, 0));
      directionOffsets.put("w", new Point(-1, 0));

      agentSizeCache = new HashMap<>();
      if (DEBUG) {
        logger.info("ObstacleManager initialized successfully");
      }
    } catch (Exception e) {
      logger.severe("Failed to initialize ObstacleManager: " + e.getMessage());
      throw new RuntimeException("ObstacleManager initialization failed", e);
    }
  }

  /**
   * Filter directions considering agent size and block
   */
  public List<String> filterDirections(
    List<String> availableDirections,
    LocalMap map,
    Point currentPos,
    int size,
    String blockDirection,
    Point previousPos,
    String previousMove
  ) {
    try {
      // Validate inputs
      if (availableDirections == null) {
        logger.severe("Available directions list is null");
        return new ArrayList<>();
      }
      if (map == null || currentPos == null) {
        logger.severe("Map or current position is null");
        return availableDirections;
      }

      if (DEBUG) {
        logger.info(
          String.format(
            "Filtering directions - Current: %s, Size: %d, Block: %s, Available: %s",
            currentPos,
            size,
            blockDirection,
            availableDirections
          )
        );
      }

      // First try normal filtering
      List<String> filteredDirections = availableDirections
        .stream()
        .filter(
          dir -> {
            try {
              boolean safe = isDirectionSafe(
                map,
                currentPos,
                dir,
                size,
                blockDirection
              );
              if (DEBUG && !safe) {
                logger.info(
                  String.format(
                    "Direction %s from %s is unsafe (size: %d, block: %s)",
                    dir,
                    currentPos,
                    size,
                    blockDirection
                  )
                );
              }
              return safe;
            } catch (Exception e) {
              logger.warning(
                String.format(
                  "Error checking direction %s: %s",
                  dir,
                  e.getMessage()
                )
              );
              return false;
            }
          }
        )
        .collect(Collectors.toList());

      if (!filteredDirections.isEmpty()) {
        if (DEBUG) {
          logger.info("Filtered directions: " + filteredDirections);
        }
        return filteredDirections;
      }

      // Handle stuck case
      if (
        previousPos != null &&
        currentPos.equals(previousPos) &&
        previousMove != null
      ) {
        if (DEBUG) {
          logger.info(
            String.format(
              "Agent stuck at %s, removing previous move %s",
              currentPos,
              previousMove
            )
          );
        }

        List<String> optimisticDirections = new ArrayList<>(
          availableDirections
        );
        optimisticDirections.remove(previousMove);

        if (!optimisticDirections.isEmpty()) {
          logger.info("Using optimistic directions: " + optimisticDirections);
          return optimisticDirections;
        }
      }

      // Fallback case
      List<String> fallbackDirections = new ArrayList<>(availableDirections);
      if (previousMove != null) {
        fallbackDirections.remove(previousMove);
      }

      if (DEBUG) {
        logger.info(
          "Using fallback directions: " +
          (
            fallbackDirections.isEmpty()
              ? availableDirections
              : fallbackDirections
          )
        );
      }

      return fallbackDirections.isEmpty()
        ? availableDirections
        : fallbackDirections;
    } catch (Exception e) {
      logger.severe("Critical error in filterDirections: " + e.getMessage());
      e.printStackTrace();
      return availableDirections; // Return original list in case of critical error
    }
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
    // Add debug logging
    System.out.println(
      "Validating move: " +
      moveDirection +
      " from " +
      agentPos +
      " with block: " +
      blockDirection
    );

    // Quick bounds check first
    Point nextPos = calculateNextPosition(agentPos, moveDirection);
    if (isPositionInvalid(nextPos, map)) {
      System.out.println("Initial position check failed: " + nextPos);
      return false;
    }

    // Get agent positions using cached patterns
    Set<Point> nextAgentPositions = getAgentPositions(nextPos, agentSize);
    Set<Point> currentAgentPositions = getAgentPositions(agentPos, agentSize);

    // Check agent positions
    for (Point pos : nextAgentPositions) {
      if (isPositionInvalid(pos, map)) {
        System.out.println("Agent position invalid: " + pos);
        return false;
      }
    }

    // If no block, we're done
    if (blockDirection == null) {
      return true;
    }

    // Check block positions
    Set<Point> currentBlockPositions = getBlockPositions(
      currentAgentPositions,
      blockDirection
    );
    Set<Point> nextBlockPositions = getBlockPositions(
      nextAgentPositions,
      blockDirection
    );

    // Add validation for block movement
    for (Point pos : nextBlockPositions) {
      if (isPositionInvalid(pos, map)) {
        System.out.println("Block position invalid: " + pos);
        return false;
      }

      // Additional check for block boundaries
      if (Math.abs(pos.y) > Math.abs(agentPos.y) + agentSize + 1) {
        System.out.println("Block movement exceeds reasonable bounds: " + pos);
        return false;
      }
    }

    // Verify the path is clear
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
    boolean invalid = map.hasObstacle(pos) || map.isOutOfBounds(pos);
    if (invalid) {
      System.out.println(
        "Position invalid: " +
        pos +
        " (obstacle: " +
        map.hasObstacle(pos) +
        ", outOfBounds: " +
        map.isOutOfBounds(pos) +
        ")"
      );
    }
    return invalid;
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

  private boolean isDirectionSafe(
    LocalMap map,
    Point currentPos,
    String direction,
    int size,
    String blockDirection
  ) {
    try {
      if (DEBUG) {
        logger.info(
          String.format(
            "Checking safety of direction %s from %s (size: %d, block: %s)",
            direction,
            currentPos,
            size,
            blockDirection
          )
        );
      }

      // Get next position
      Point nextPos = calculateNextPosition(currentPos, direction);

      // Check agent positions
      Set<Point> nextAgentPositions = getAgentPositions(nextPos, size);
      Set<Point> currentAgentPositions = getAgentPositions(currentPos, size);

      // Check for obstacles in agent positions
      for (Point pos : nextAgentPositions) {
        if (isPositionInvalid(pos, map)) {
          if (DEBUG) {
            logger.info(
              String.format("Position %s is invalid for agent movement", pos)
            );
          }
          return false;
        }
      }

      // If no block, we're done
      if (blockDirection == null) {
        return true;
      }

      // Check block positions
      Set<Point> currentBlockPositions = getBlockPositions(
        currentAgentPositions,
        blockDirection
      );
      Set<Point> nextBlockPositions = getBlockPositions(
        nextAgentPositions,
        blockDirection
      );

      // Check for obstacles in block positions
      for (Point pos : nextBlockPositions) {
        if (isPositionInvalid(pos, map)) {
          if (DEBUG) {
            logger.info(
              String.format("Position %s is invalid for block movement", pos)
            );
          }
          return false;
        }
      }

      return !hasObstacleInPath(
        currentAgentPositions,
        nextAgentPositions,
        currentBlockPositions,
        nextBlockPositions,
        map
      );
    } catch (Exception e) {
      logger.warning("Error checking direction safety: " + e.getMessage());
      return false; // Safer to consider direction unsafe on error
    }
  }
}
