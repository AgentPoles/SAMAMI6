package jason.eis.movements.collision;

import jason.eis.LocalMap;
import jason.eis.Point;
import java.util.*;
import java.util.logging.Logger;

public class ForcedDirectionChange implements CollisionResolver {
  private static final Logger logger = Logger.getLogger(
    ForcedDirectionChange.class.getName()
  );
  private static final boolean DEBUG = false;
  private static final int DIRECTION_CHANGE_THRESHOLD = 7;

  // Track consecutive moves in the same direction for each agent
  private final Map<String, DirectionState> agentDirections = new HashMap<>();

  private static class DirectionState {
    String currentDirection;
    int consecutiveMoves;

    DirectionState(String direction) {
      this.currentDirection = direction;
      this.consecutiveMoves = 1;
    }

    void updateDirection(String newDirection) {
      if (newDirection.equals(currentDirection)) {
        consecutiveMoves++;
      } else {
        currentDirection = newDirection;
        consecutiveMoves = 1;
      }
    }

    boolean needsDirectionChange() {
      return consecutiveMoves >= DIRECTION_CHANGE_THRESHOLD;
    }
  }

  @Override
  public CollisionResolution checkCollision(
    String agentName,
    Point currentPos,
    String plannedDirection,
    LocalMap map,
    int agentSize,
    String blockDirection,
    List<String> availableDirections
  ) {
    try {
      if (
        !isValidInput(
          agentName,
          currentPos,
          plannedDirection,
          availableDirections
        )
      ) {
        return null;
      }

      // Update direction state
      DirectionState state = agentDirections.computeIfAbsent(
        agentName,
        k -> new DirectionState(plannedDirection)
      );

      if (plannedDirection != null) {
        state.updateDirection(plannedDirection);
      }

      // Check if we need to force a direction change
      if (state.needsDirectionChange()) {
        debug(
          "[%s] Forcing direction change after %d consecutive moves in direction %s",
          agentName,
          state.consecutiveMoves,
          state.currentDirection
        );

        String oppositeDirection = getOppositeDirection(state.currentDirection);

        // Reset the state after forcing a change
        state.currentDirection = oppositeDirection;
        state.consecutiveMoves = 1;

        // Only return the opposite direction if it's available
        if (availableDirections.contains(oppositeDirection)) {
          return new CollisionResolution(oppositeDirection, "FORCED_CHANGE");
        }

        // If opposite direction isn't available, try perpendicular directions
        List<String> perpendicularDirs = getPerpendicularDirections(
          state.currentDirection
        );
        for (String dir : perpendicularDirs) {
          if (availableDirections.contains(dir)) {
            return new CollisionResolution(dir, "FORCED_CHANGE");
          }
        }
      }

      return null;
    } catch (Exception e) {
      logger.warning("Error in ForcedDirectionChange: " + e.getMessage());
      return null;
    }
  }

  private boolean isValidInput(
    String agentName,
    Point currentPos,
    String plannedDirection,
    List<String> availableDirections
  ) {
    return (
      agentName != null &&
      currentPos != null &&
      availableDirections != null &&
      !availableDirections.isEmpty()
    );
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

  private List<String> getPerpendicularDirections(String direction) {
    if (direction == null) return Collections.emptyList();
    switch (direction) {
      case "n":
      case "s":
        return Arrays.asList("e", "w");
      case "e":
      case "w":
        return Arrays.asList("n", "s");
      default:
        return Collections.emptyList();
    }
  }

  private void debug(String format, Object... args) {
    if (DEBUG) {
      logger.fine(String.format(format, args));
    }
  }

  // Method to clear state for an agent (useful when agent reaches target or path is cleared)
  public void clearAgentState(String agentName) {
    if (agentName != null) {
      agentDirections.remove(agentName);
    }
  }
}
