package jason.eis.movements;

import jason.eis.LocalMap;
import jason.eis.Point;
import jason.eis.movements.collision.CollisionResolution;
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
  private static final int AWARENESS_ZONE = 4;
  private static final double CRITICAL_DISTANCE = 1.5;

  // Shared states
  private final BaseCollisionState baseState;
  private final StuckState stuckState;

  // Handlers
  private final StuckHandler stuckHandler;
  private final OscillationHandler oscillationHandler;
  private final Map<String, AgentState> agentStates;

  public AgentCollisionHandler() {
    this.baseState = new BaseCollisionState();
    this.stuckState = new StuckState();
    this.stuckHandler = new StuckHandler(baseState, stuckState);
    this.oscillationHandler = new OscillationHandler();
    this.agentStates = new HashMap<>();
  }

  private static class AgentState {
    Point currentPos;
    Point previousPos;
    String lastDirection;
    long lastMoveTime;
    int size;

    AgentState(Point pos, String direction, int size) {
      this.currentPos = pos;
      this.lastDirection = direction;
      this.size = size;
      this.lastMoveTime = System.currentTimeMillis();
    }

    void updateState(Point newPos, String direction, int newSize) {
      this.previousPos = this.currentPos;
      this.currentPos = newPos;
      this.lastDirection = direction;
      this.size = newSize;
      this.lastMoveTime = System.currentTimeMillis();
    }
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

      // Update agent state
      updateState(agentId, currentPos, intendedDirection, size);
      AgentState state = agentStates.get(agentId);

      // First check for stuck condition (highest priority)
      if (isStuck(state)) {
        String stuckResolution = stuckHandler.resolveStuck(
          agentId,
          map,
          availableDirections
        );
        if (stuckResolution != null) {
          return new CollisionResolution(stuckResolution, "STUCK");
        }
      }

      // Then check for oscillation
      String oscillationResolution = oscillationHandler.resolveOscillation(
        agentId,
        currentPos,
        intendedDirection,
        availableDirections,
        size,
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

      // No collision detected
      return null;
    } catch (Exception e) {
      logger.severe("Error in resolveCollision: " + e.getMessage());
      return null;
    }
  }

  private void updateState(
    String agentId,
    Point currentPos,
    String direction,
    int size
  ) {
    AgentState state = agentStates.get(agentId);
    if (state == null) {
      state = new AgentState(currentPos, direction, size);
      agentStates.put(agentId, state);
    } else {
      state.updateState(currentPos, direction, size);
    }
  }

  private boolean isStuck(AgentState state) {
    if (state == null || state.previousPos == null) {
      return false;
    }
    return (
      state.currentPos.equals(state.previousPos) &&
      (System.currentTimeMillis() - state.lastMoveTime) > 1000
    ); // 1 second threshold
  }

  // Helper method to clean up old states periodically
  public void cleanup() {
    long currentTime = System.currentTimeMillis();
    agentStates
      .entrySet()
      .removeIf(
        entry -> (currentTime - entry.getValue().lastMoveTime) > 10000 // 10 seconds
      );
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
