package jason.eis.movements.collision.handlers;

import jason.eis.Point;
import java.util.*;
import java.util.logging.Logger;
import java.util.stream.Collectors;

public class OscillationHandler {
  private static final Logger logger = Logger.getLogger(
    OscillationHandler.class.getName()
  );
  private static final boolean DEBUG = true;
  private static final long OSCILLATION_TIMEOUT = 5000;
  private static final int OSCILLATION_THRESHOLD = 3;

  private final Map<String, OscillationState> agentStates = new HashMap<>();

  private static class OscillationState {
    List<Point> positions;
    List<String> directions;
    int size;
    int twoStepPatternCount;
    int threeStepPatternCount;
    long lastMoveTime;

    OscillationState(Point startPos, String direction, int size) {
      this.positions = new ArrayList<>();
      this.directions = new ArrayList<>();
      this.positions.add(startPos);
      this.directions.add(direction);
      this.size = size;
      this.twoStepPatternCount = 0;
      this.threeStepPatternCount = 0;
      this.lastMoveTime = System.currentTimeMillis();
    }

    boolean isOscillating() {
      return (
        (
          twoStepPatternCount >= OSCILLATION_THRESHOLD ||
          threeStepPatternCount >= OSCILLATION_THRESHOLD
        ) &&
        (System.currentTimeMillis() - lastMoveTime) < OSCILLATION_TIMEOUT
      );
    }

    void addMove(Point pos, String direction) {
      positions.add(pos);
      directions.add(direction);

      // Keep only last 6 positions and directions (for 3-step pattern checking)
      if (positions.size() > 6) {
        positions.remove(0);
        directions.remove(0);
      }
      lastMoveTime = System.currentTimeMillis();
    }

    String getOscillationType() {
      if (twoStepPatternCount >= OSCILLATION_THRESHOLD) return "TWO_STEP";
      if (threeStepPatternCount >= OSCILLATION_THRESHOLD) return "THREE_STEP";
      return null;
    }
  }

  public String resolveOscillation(
    String agentId,
    Point currentPos,
    String intendedDirection,
    List<String> availableDirections,
    int size,
    String blockAttachment
  ) {
    try {
      if (DEBUG) {
        logger.info(
          String.format(
            "Checking oscillation for agent %s at %s moving %s",
            agentId,
            currentPos,
            intendedDirection
          )
        );
      }

      OscillationState state = agentStates.get(agentId);
      if (state == null) {
        state = new OscillationState(currentPos, intendedDirection, size);
        agentStates.put(agentId, state);
        return null;
      }

      // Check if size changed
      if (state.size != size) {
        agentStates.remove(agentId);
        return null;
      }

      // Add new move to state
      state.addMove(currentPos, intendedDirection);

      // Check for 2-step oscillation
      if (state.positions.size() >= 2) {
        if (isTwoStepPattern(state)) {
          state.twoStepPatternCount++;
          if (DEBUG) {
            logger.info(
              String.format(
                "2-step pattern detected (%d times): %s",
                state.twoStepPatternCount,
                state.positions.subList(
                  state.positions.size() - 2,
                  state.positions.size()
                )
              )
            );
          }
        }
      }

      // Check for 3-step oscillation
      if (state.positions.size() >= 3) {
        if (isThreeStepPattern(state)) {
          state.threeStepPatternCount++;
          if (DEBUG) {
            logger.info(
              String.format(
                "3-step pattern detected (%d times): %s",
                state.threeStepPatternCount,
                state.positions.subList(
                  state.positions.size() - 3,
                  state.positions.size()
                )
              )
            );
          }
        }
      }

      if (state.isOscillating()) {
        return resolveOscillatingMove(
          state,
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

  private boolean isTwoStepPattern(OscillationState state) {
    if (state.positions.size() < 2) return false;

    Point lastPos = state.positions.get(state.positions.size() - 1);
    Point secondLastPos = state.positions.get(state.positions.size() - 2);

    // Check if we're alternating between two positions
    return (
      lastPos.equals(state.positions.get(state.positions.size() - 1)) &&
      secondLastPos.equals(state.positions.get(state.positions.size() - 2))
    );
  }

  private boolean isThreeStepPattern(OscillationState state) {
    if (state.positions.size() < 6) return false; // Need at least 6 positions to check for repeat

    List<Point> currentPattern = state.positions.subList(
      state.positions.size() - 3,
      state.positions.size()
    );
    List<String> currentDirPattern = state.directions.subList(
      state.directions.size() - 3,
      state.directions.size()
    );

    List<Point> previousPattern = state.positions.subList(
      state.positions.size() - 6,
      state.positions.size() - 3
    );
    List<String> previousDirPattern = state.directions.subList(
      state.directions.size() - 6,
      state.directions.size() - 3
    );

    return (
      currentPattern.equals(previousPattern) &&
      currentDirPattern.equals(previousDirPattern)
    );
  }

  private String resolveOscillatingMove(
    OscillationState state,
    List<String> availableDirections,
    String blockAttachment
  ) {
    try {
      if (availableDirections.isEmpty()) {
        return null;
      }

      String oscillationType = state.getOscillationType();
      if (DEBUG) {
        logger.info("Resolving " + oscillationType + " oscillation");
      }

      Set<String> oscillatingDirs = new HashSet<>(state.directions);

      // For 2-step oscillation, prefer perpendicular directions
      if ("TWO_STEP".equals(oscillationType)) {
        String lastDir = state.directions.get(state.directions.size() - 1);
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

      // For both types, try non-oscillating directions
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
        perpDirs.add("w");
        perpDirs.add("e");
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
