package jason.eis.movements;

import jason.eis.LocalMap;
import jason.eis.Point;
import jason.eis.movements.collision.CollisionResolution;
import jason.eis.movements.collision.data.*;
import jason.eis.movements.collision.handlers.*;
import java.util.*;

public class AgentCollisionHandler {
  // Constants for collision detection
  private static final int AWARENESS_ZONE = 4;
  private static final double CRITICAL_DISTANCE = 1.5;

  // Shared states
  private final BaseCollisionState baseState;
  private final StuckState stuckState;

  // Handlers
  private final StuckHandler stuckHandler;

  public AgentCollisionHandler() {
    this.baseState = new BaseCollisionState();
    this.stuckState = new StuckState();
    this.stuckHandler = new StuckHandler(baseState, stuckState);
  }

  /**
   * Main method to handle collisions and provide resolution
   * @param agentId The agent requesting collision resolution
   * @param currentPos Current position of the agent
   * @param intendedDirection Direction the agent wants to move
   * @param map Current state of the environment
   * @param size Size of the agent (1 for single, 2+ for block attached)
   * @param blockAttachment Direction of block attachment if any
   * @param availableDirections List of available directions for the agent
   * @return Resolved direction or null if no resolution needed
   */
  public CollisionResolution resolveCollision(
    String agentId,
    Point currentPos,
    String intendedDirection,
    LocalMap map,
    int size,
    String blockAttachment,
    List<String> availableDirections
  ) {
    // Update states first
    updateState(agentId, currentPos, intendedDirection, size, blockAttachment);

    // Check if agent is stuck
    if (stuckState.isStuck(agentId)) {
      List<String> availableDirs = getAvailableDirections(
        map,
        currentPos,
        size,
        blockAttachment
      );
      String resolvedDirection = stuckHandler.resolveStuck(
        agentId,
        map,
        availableDirs
      );

      if (resolvedDirection != null) {
        return new CollisionResolution(resolvedDirection, "STUCK");
      }
    }

    // No collision detected
    return null;
  }

  private void updateState(
    String agentId,
    Point position,
    String intendedDirection,
    int size,
    String blockAttachment
  ) {
    baseState.updateAgentState(agentId, size, blockAttachment);
    stuckHandler.updateState(agentId, position, intendedDirection);
  }

  private List<String> getAvailableDirections(
    LocalMap map,
    Point pos,
    int size,
    String blockAttachment
  ) {
    List<String> available = new ArrayList<>();

    // Check each cardinal direction
    for (String dir : Arrays.asList("n", "s", "e", "w")) {
      if (isDirectionAvailable(map, pos, dir, size, blockAttachment)) {
        available.add(dir);
      }
    }

    return available;
  }

  private boolean isDirectionAvailable(
    LocalMap map,
    Point pos,
    String direction,
    int size,
    String blockAttachment
  ) {
    Point nextPos = calculateNextPosition(pos, direction);

    // Basic checks
    if (map.isOutOfBounds(nextPos) || map.hasObstacle(nextPos)) {
      return false;
    }

    // Size and attachment checks
    if (size > 1 || blockAttachment != null) {
      // Check if movement would cause block rotation
      if (
        blockAttachment != null && !isValidBlockMove(direction, blockAttachment)
      ) {
        // Additional space needed for rotation
        if (!hasRotationSpace(map, pos, direction)) {
          return false;
        }
      }

      // Check space for larger size
      for (Point occupiedPoint : getOccupiedPoints(nextPos, size, direction)) {
        if (
          map.isOutOfBounds(occupiedPoint) || map.hasObstacle(occupiedPoint)
        ) {
          return false;
        }
      }
    }

    return true;
  }

  private boolean isValidBlockMove(
    String moveDirection,
    String blockAttachment
  ) {
    // Moving parallel to attachment or in attachment direction is valid
    switch (blockAttachment) {
      case "n":
      case "s":
        return moveDirection.equals("n") || moveDirection.equals("s");
      case "e":
      case "w":
        return moveDirection.equals("e") || moveDirection.equals("w");
      default:
        return true;
    }
  }

  private boolean hasRotationSpace(LocalMap map, Point pos, String direction) {
    // Check 3x3 area around next position for rotation space
    Point nextPos = calculateNextPosition(pos, direction);
    for (int dx = -1; dx <= 1; dx++) {
      for (int dy = -1; dy <= 1; dy++) {
        Point checkPos = new Point(nextPos.x + dx, nextPos.y + dy);
        if (map.isOutOfBounds(checkPos) || map.hasObstacle(checkPos)) {
          return false;
        }
      }
    }
    return true;
  }

  private List<Point> getOccupiedPoints(
    Point basePos,
    int size,
    String direction
  ) {
    List<Point> points = new ArrayList<>();
    points.add(basePos);

    if (size > 1) {
      // Add additional points based on size and direction
      switch (direction) {
        case "n":
          points.add(new Point(basePos.x, basePos.y - 1));
          break;
        case "s":
          points.add(new Point(basePos.x, basePos.y + 1));
          break;
        case "e":
          points.add(new Point(basePos.x + 1, basePos.y));
          break;
        case "w":
          points.add(new Point(basePos.x - 1, basePos.y));
          break;
      }
    }

    return points;
  }

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

  public void resetState(String agentId) {
    baseState.resetAgentState(agentId);
    stuckState.resetStuckState(agentId);
  }
}
