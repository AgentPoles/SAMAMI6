package jason.eis.movements.collision.handlers;

import jason.eis.LocalMap;
import jason.eis.Point;
import java.util.*;
import java.util.logging.Logger;
import java.util.stream.Collectors;

public class OscillationHandler {
  private static final Logger logger = Logger.getLogger(
    OscillationHandler.class.getName()
  );
  private static final boolean DEBUG = true;

  public String resolveOscillation(
    LocalMap localMap,
    Point currentPos,
    String intendedDirection,
    List<String> availableDirections,
    String blockAttachment
  ) {
    try {
      if (DEBUG) {
        logger.info(
          String.format(
            "Checking oscillation at %s moving %s",
            currentPos,
            intendedDirection
          )
        );
      }

      // Check if we're oscillating
      if (localMap.isOscillating()) {
        return resolveOscillatingMove(
          localMap,
          availableDirections,
          blockAttachment
        );
      }

      return null;
    } catch (Exception e) {
      logger.severe("Error in resolveOscillation: " + e.getMessage());
      e.printStackTrace();
      return null;
    }
  }

  private String resolveOscillatingMove(
    LocalMap localMap,
    List<String> availableDirections,
    String blockAttachment
  ) {
    try {
      if (availableDirections.isEmpty()) {
        return null;
      }

      Set<String> oscillatingDirs = localMap.getOscillatingDirections();

      // For 2-step oscillation, prefer perpendicular directions
      String lastDir = localMap.getLastDirection();
      if (lastDir != null) {
        List<String> perpDirs = getPerpendicularDirections(
          lastDir,
          blockAttachment
        );
        List<String> validPerpDirs = perpDirs
          .stream()
          .filter(availableDirections::contains)
          .collect(Collectors.toList());

        if (!validPerpDirs.isEmpty()) {
          String chosen = validPerpDirs.get(0);
          if (DEBUG) logger.info("Chose perpendicular direction: " + chosen);
          return chosen;
        }
      }

      // Try non-oscillating directions
      List<String> validDirs = availableDirections
        .stream()
        .filter(dir -> !oscillatingDirs.contains(dir))
        .collect(Collectors.toList());

      if (!validDirs.isEmpty()) {
        String chosen = validDirs.get(0);
        if (DEBUG) logger.info("Chose non-oscillating direction: " + chosen);
        return chosen;
      }

      // Last resort: any available direction
      String chosen = availableDirections.get(0);
      if (DEBUG) logger.info("Chose fallback direction: " + chosen);
      return chosen;
    } catch (Exception e) {
      logger.severe("Error in resolveOscillatingMove: " + e.getMessage());
      e.printStackTrace();
      return availableDirections.isEmpty() ? null : availableDirections.get(0);
    }
  }

  private List<String> getPerpendicularDirections(
    String direction,
    String blockAttachment
  ) {
    List<String> perpDirs = new ArrayList<>();
    switch (direction) {
      case "n":
      case "s":
        perpDirs.add("e");
        perpDirs.add("w");
        break;
      case "e":
      case "w":
        perpDirs.add("n");
        perpDirs.add("s");
        break;
    }

    // If block attached, filter out conflicting directions
    if (blockAttachment != null) {
      perpDirs.removeIf(
        dir ->
          dir.equals(blockAttachment) ||
          dir.equals(getOppositeDirection(blockAttachment))
      );
    }

    return perpDirs;
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
}
