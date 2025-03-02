package jason.eis.movements.collision.handlers;

import jason.eis.LocalMap;
import jason.eis.Point;
import jason.eis.movements.collision.data.*;
import java.util.*;

public class StuckHandler {
  private final BaseCollisionState baseState;
  private final StuckState stuckState;
  private final Random random = new Random();

  public StuckHandler(BaseCollisionState baseState, StuckState stuckState) {
    this.baseState = baseState;
    this.stuckState = stuckState;
  }

  public void updateState(
    String agentId,
    Point position,
    String intendedDirection
  ) {
    MovementRecord lastMove = baseState.getLastMovement(agentId);
    // Record the new movement
    baseState.recordMovement(agentId, position, intendedDirection);
    // Check if position hasn't changed
    if (lastMove != null && lastMove.getPosition().equals(position)) {
      stuckState.incrementStuck(agentId, intendedDirection);
    } else {
      stuckState.resetStuckState(agentId);
    }
  }

  public String resolveStuck(
    String agentId,
    LocalMap map,
    List<String> availableDirections
  ) {
    if (!stuckState.isStuck(agentId)) {
      return null;
    }

    // Get agent properties
    int agentSize = baseState.getAgentSize(agentId);
    String blockAttachment = baseState.getBlockAttachment(agentId);

    // Get movement history
    List<MovementRecord> history = baseState.getMovementHistory(agentId);
    Set<String> triedDirections = stuckState.getTriedDirections(agentId);

    // If we have a block attached, use block-aware resolution
    if (agentSize > 1 || blockAttachment != null) {
      return resolveBlockStuck(
        agentId,
        map,
        availableDirections,
        blockAttachment
      );
    }

    // For single agents, try these strategies in order:

    // 1. Try opposite of last intended direction if not tried
    if (!history.isEmpty()) {
      String lastIntended = history.get(0).getIntendedDirection();
      String oppositeDir = getOppositeDirection(lastIntended);

      if (
        oppositeDir != null &&
        !triedDirections.contains(oppositeDir) &&
        availableDirections.contains(oppositeDir)
      ) {
        return oppositeDir;
      }
    }

    // 2. Try perpendicular directions if available
    List<String> perpendicularDirs = getPerpendicularDirections(
      history,
      availableDirections
    );
    if (!perpendicularDirs.isEmpty()) {
      return perpendicularDirs.get(random.nextInt(perpendicularDirs.size()));
    }

    // 3. Try any untried available direction
    List<String> untriedDirs = new ArrayList<>(availableDirections);
    untriedDirs.removeAll(triedDirections);
    if (!untriedDirs.isEmpty()) {
      return untriedDirs.get(random.nextInt(untriedDirs.size()));
    }

    // 4. If all else fails, pick random available direction
    if (!availableDirections.isEmpty()) {
      return availableDirections.get(
        random.nextInt(availableDirections.size())
      );
    }

    // No available directions
    return null;
  }

  private String resolveBlockStuck(
    String agentId,
    LocalMap map,
    List<String> availableDirections,
    String blockAttachment
  ) {
    // For agents with blocks, we need to be more careful

    // 1. Prefer moving in directions that don't require rotation
    List<String> safeDirections = getSafeDirectionsWithBlock(
      availableDirections,
      blockAttachment
    );

    if (!safeDirections.isEmpty()) {
      return safeDirections.get(random.nextInt(safeDirections.size()));
    }

    // 2. If we must rotate, ensure we have space
    List<String> rotationSafeDirections = getRotationSafeDirections(
      map,
      availableDirections,
      baseState.getLastMovement(agentId).getPosition()
    );

    if (!rotationSafeDirections.isEmpty()) {
      return rotationSafeDirections.get(
        random.nextInt(rotationSafeDirections.size())
      );
    }

    // 3. If no safe directions, use any available direction
    if (!availableDirections.isEmpty()) {
      return availableDirections.get(
        random.nextInt(availableDirections.size())
      );
    }

    return null;
  }

  private List<String> getSafeDirectionsWithBlock(
    List<String> availableDirections,
    String blockAttachment
  ) {
    List<String> safeDirections = new ArrayList<>();

    for (String dir : availableDirections) {
      // Safe if moving parallel to attachment or in attachment direction
      if (
        isParallelToAttachment(dir, blockAttachment) ||
        dir.equals(blockAttachment)
      ) {
        safeDirections.add(dir);
      }
    }

    return safeDirections;
  }

  private List<String> getRotationSafeDirections(
    LocalMap map,
    List<String> availableDirections,
    Point position
  ) {
    List<String> safeDirections = new ArrayList<>();

    for (String dir : availableDirections) {
      // Check if we have space to rotate in this direction
      if (hasRotationSpace(map, position, dir)) {
        safeDirections.add(dir);
      }
    }

    return safeDirections;
  }

  private boolean hasRotationSpace(
    LocalMap map,
    Point position,
    String direction
  ) {
    // Check surrounding cells for rotation space
    // This is a simplified check - might need more complex logic
    Point nextPos = calculateNextPosition(position, direction);

    // Check immediate neighbors of next position
    for (int dx = -1; dx <= 1; dx++) {
      for (int dy = -1; dy <= 1; dy++) {
        Point checkPos = new Point(nextPos.x + dx, nextPos.y + dy);
        if (map.hasObstacle(checkPos) || map.isOutOfBounds(checkPos)) {
          return false;
        }
      }
    }

    return true;
  }

  private boolean isParallelToAttachment(String direction, String attachment) {
    if (attachment == null) return true;

    switch (attachment) {
      case "n":
      case "s":
        return direction.equals("n") || direction.equals("s");
      case "e":
      case "w":
        return direction.equals("e") || direction.equals("w");
      default:
        return false;
    }
  }

  private List<String> getPerpendicularDirections(
    List<MovementRecord> history,
    List<String> availableDirections
  ) {
    if (history.isEmpty()) return new ArrayList<>();

    String lastDir = history.get(0).getIntendedDirection();
    List<String> perpendicular = new ArrayList<>();

    switch (lastDir) {
      case "n":
      case "s":
        if (availableDirections.contains("e")) perpendicular.add("e");
        if (availableDirections.contains("w")) perpendicular.add("w");
        break;
      case "e":
      case "w":
        if (availableDirections.contains("n")) perpendicular.add("n");
        if (availableDirections.contains("s")) perpendicular.add("s");
        break;
    }

    return perpendicular;
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
