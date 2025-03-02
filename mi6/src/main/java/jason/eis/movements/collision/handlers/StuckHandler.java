package jason.eis.movements.collision.handlers;

import jason.eis.LocalMap;
import jason.eis.Point;
import java.util.*;

public class StuckHandler {
  private final Random random = new Random();

  public StuckHandler() {
    // No dependencies needed
  }

  public String resolveStuck(
    String agentName,
    LocalMap map,
    List<String> availableDirections
  ) {
    if (!map.isStuck()) {
      return null;
    }

    int agentSize = map.getAgentSize();
    String blockAttachment = map.getBlockAttachment();
    Set<String> triedDirections = map.getTriedDirections();

    if (agentSize > 1 || blockAttachment != null) {
      return resolveBlockStuck(map, availableDirections, blockAttachment);
    }

    String lastDir = map.getLastDirection();
    if (lastDir != null) {
      String oppositeDir = getOppositeDirection(lastDir);
      if (
        oppositeDir != null &&
        !triedDirections.contains(oppositeDir) &&
        availableDirections.contains(oppositeDir)
      ) {
        return oppositeDir;
      }
    }

    List<String> perpendicularDirs = getPerpendicularDirections(
      map.getLastDirection(),
      availableDirections
    );
    if (!perpendicularDirs.isEmpty()) {
      return perpendicularDirs.get(random.nextInt(perpendicularDirs.size()));
    }

    List<String> untriedDirs = new ArrayList<>(availableDirections);
    untriedDirs.removeAll(triedDirections);
    if (!untriedDirs.isEmpty()) {
      return untriedDirs.get(random.nextInt(untriedDirs.size()));
    }

    if (!availableDirections.isEmpty()) {
      return availableDirections.get(
        random.nextInt(availableDirections.size())
      );
    }

    return null;
  }

  private String resolveBlockStuck(
    LocalMap map,
    List<String> availableDirections,
    String blockAttachment
  ) {
    List<String> safeDirections = getSafeDirectionsWithBlock(
      availableDirections,
      blockAttachment
    );

    if (!safeDirections.isEmpty()) {
      return safeDirections.get(random.nextInt(safeDirections.size()));
    }

    List<String> rotationSafeDirections = getRotationSafeDirections(
      map,
      availableDirections,
      map.getCurrentPosition()
    );

    if (!rotationSafeDirections.isEmpty()) {
      return rotationSafeDirections.get(
        random.nextInt(rotationSafeDirections.size())
      );
    }

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
    Point nextPos = calculateNextPosition(position, direction);

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
    String lastDir,
    List<String> availableDirections
  ) {
    if (lastDir == null) return new ArrayList<>();

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
