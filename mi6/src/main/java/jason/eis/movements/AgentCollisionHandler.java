package jason.eis.movements;

import jason.eis.LocalMap;
import jason.eis.Point;
import jason.eis.movements.collision.CollisionResolution;
import jason.eis.movements.collision.ForcedDirectionChange;
import jason.eis.movements.collision.data.*;
import jason.eis.movements.collision.handlers.*;
import java.util.*;
import java.util.logging.Logger;

public class AgentCollisionHandler {
  private static final Logger logger = Logger.getLogger(
    AgentCollisionHandler.class.getName()
  );
  private static final boolean DEBUG = true;

  // Constants for collision detection
  private static final int AWARENESS_ZONE = 1;
  private static final double CRITICAL_DISTANCE = 1;

  // Handlers
  private final StuckHandler stuckHandler;
  private final OscillationHandler oscillationHandler;
  private final ForcedDirectionChange forcedDirectionHandler;
  private final AgentUntangler untangler;

  public AgentCollisionHandler() {
    this.stuckHandler = new StuckHandler();
    this.oscillationHandler = new OscillationHandler();
    this.forcedDirectionHandler = new ForcedDirectionChange();
    this.untangler = new AgentUntangler();
  }

  /**
   * Main method to handle collisixons and provide resolution
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
    try {
      if (DEBUG) {
        logger.info(
          String.format(
            "Resolving collision for agent %s at %s, size %d, block %s",
            agentId,
            currentPos,
            size,
            blockAttachment
          )
        );
      }

      // Try untangling first (highest priority for agent-agent conflicts)
      String untangleDirection = untangler.untangle(agentId, map);
      if (untangleDirection != null) {
        if (DEBUG) {
          logger.info(
            String.format(
              "Untangling agent %s with direction %s",
              agentId,
              untangleDirection
            )
          );
        }
        return new CollisionResolution(untangleDirection, "UNTANGLE");
      }

      // Check for forced direction change
      CollisionResolution forcedChange = forcedDirectionHandler.checkCollision(
        agentId,
        currentPos,
        intendedDirection,
        map,
        size,
        blockAttachment,
        availableDirections
      );

      if (forcedChange != null) {
        if (DEBUG) {
          logger.info(
            String.format(
              "Forced direction change for agent %s, new direction: %s",
              agentId,
              forcedChange.getDirection()
            )
          );
        }
        return forcedChange;
      }

      // Check for stuck condition
      if (map.isStuck()) {
        String stuckResolution = stuckHandler.resolveStuck(
          agentId,
          map,
          availableDirections
        );
        if (stuckResolution != null) {
          return new CollisionResolution(stuckResolution, "STUCK");
        }
      }

      // Check for oscillation
      String oscillationResolution = oscillationHandler.resolveOscillation(
        map,
        currentPos,
        intendedDirection,
        availableDirections,
        blockAttachment
      );
      if (oscillationResolution != null) {
        if (DEBUG) {
          logger.info(
            String.format(
              "Oscillation detected for agent %s, resolving with direction %s",
              agentId,
              oscillationResolution
            )
          );
        }
        return new CollisionResolution(oscillationResolution, "OSCILLATION");
      }

      return null;
    } catch (Exception e) {
      logger.severe("Error in resolveCollision: " + e.getMessage());
      return null;
    }
  }

  // Helper methods for movement validation
  private List<String> getAvailableDirections(
    LocalMap map,
    Point pos,
    int size,
    String blockAttachment
  ) {
    List<String> available = new ArrayList<>();
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
    if (map.isOutOfBounds(nextPos) || map.hasStaticOrDynamicObstacle(nextPos)) {
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
          map.isOutOfBounds(occupiedPoint) ||
          map.hasStaticOrDynamicObstacle(occupiedPoint)
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
        if (
          map.isOutOfBounds(checkPos) ||
          map.hasStaticOrDynamicObstacle(checkPos)
        ) {
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
}
