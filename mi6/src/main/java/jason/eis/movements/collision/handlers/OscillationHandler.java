package jason.eis.movements.collision.handlers;

import jason.eis.Point;
import java.util.*;
import java.util.concurrent.ConcurrentHashMap;
import java.util.logging.Logger;
import java.util.stream.Collectors;

public class OscillationHandler {
  private static final Logger logger = Logger.getLogger(
    OscillationHandler.class.getName()
  );
  private static final boolean DEBUG = true;
  private static final long OSCILLATION_TIMEOUT = 5000;
  private static final int OSCILLATION_THRESHOLD = 3;
  private static final int PATTERN_HISTORY_SIZE = 9;
  private static final long PATTERN_TIMEOUT = 2000; // 2 seconds

  private final Map<String, OscillationState> agentStates = new HashMap<>();
  private final Map<String, List<String>> directionHistory = new ConcurrentHashMap<>();

  // Define MovementRecord as a static inner class
  private static class MovementRecord {
    final Point position;
    final String direction;
    final long timestamp;

    MovementRecord(Point pos, String dir) {
      this.position = pos;
      this.direction = dir;
      this.timestamp = System.currentTimeMillis();
    }
  }

  private static class OscillationState {
    Deque<MovementRecord> movements = new ArrayDeque<>();
    int patternCount = 0;
    long lastUpdateTime = System.currentTimeMillis();

    void addMovement(Point pos, String dir) {
      movements.addLast(new MovementRecord(pos, dir));
      if (movements.size() > PATTERN_HISTORY_SIZE) {
        movements.removeFirst();
      }
      lastUpdateTime = System.currentTimeMillis();
    }

    void resetPatternCount() {
      patternCount = 0;
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
        state = new OscillationState();
        agentStates.put(agentId, state);
        return null;
      }

      // Check if size changed
      if (
        state.movements.size() > 0 &&
        state.movements.getLast().position.x != currentPos.x
      ) {
        agentStates.remove(agentId);
        return null;
      }

      // Add new move to state
      state.addMovement(currentPos, intendedDirection);

      // Check for oscillation
      if (detectPattern(state.movements)) {
        state.patternCount++;
        if (DEBUG) {
          logger.info(
            String.format(
              "Oscillation detected (%d times): %s",
              state.patternCount,
              state
                .movements.stream()
                .map(m -> m.position.toString())
                .collect(Collectors.joining(", "))
            )
          );
        }
      } else {
        state.resetPatternCount();
      }

      if (
        state.patternCount >= OSCILLATION_THRESHOLD &&
        (System.currentTimeMillis() - state.lastUpdateTime) < PATTERN_TIMEOUT
      ) {
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

  private boolean detectPattern(Deque<MovementRecord> movements) {
    if (movements.size() < 4) return false;

    List<MovementRecord> recent = new ArrayList<>(movements);

    // Two-step pattern check with null safety
    String lastDir = recent.get(recent.size() - 1).direction;
    String prevDir = recent.get(recent.size() - 2).direction;
    if (lastDir != null && prevDir != null && isOpposite(lastDir, prevDir)) {
      return true;
    }

    // Three-step pattern check
    if (movements.size() >= 9) {
      List<String> lastThree = new ArrayList<>();
      List<String> middleThree = new ArrayList<>();
      List<String> firstThree = new ArrayList<>();

      // Manually collect non-null directions into lists
      for (int i = recent.size() - 3; i < recent.size(); i++) {
        String dir = recent.get(i).direction;
        if (dir == null) return false; // Skip pattern check if any direction is null
        lastThree.add(dir);
      }
      for (int i = recent.size() - 6; i < recent.size() - 3; i++) {
        String dir = recent.get(i).direction;
        if (dir == null) return false;
        middleThree.add(dir);
      }
      for (int i = recent.size() - 9; i < recent.size() - 6; i++) {
        String dir = recent.get(i).direction;
        if (dir == null) return false;
        firstThree.add(dir);
      }

      return (
        lastThree.equals(middleThree) ||
        lastThree.equals(firstThree) ||
        middleThree.equals(firstThree)
      );
    }

    return false;
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

      String oscillationType = detectPattern(state.movements)
        ? "TWO_STEP"
        : "THREE_STEP";
      if (DEBUG) {
        logger.info("Resolving " + oscillationType + " oscillation");
      }

      Set<String> oscillatingDirs = new HashSet<>(
        state
          .movements.stream()
          .map(m -> m.direction)
          .collect(Collectors.toList())
      );

      // For 2-step oscillation, prefer perpendicular directions
      if ("TWO_STEP".equals(oscillationType)) {
        String lastDir = state.movements.getLast().direction;
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

  private boolean isOpposite(String dir1, String dir2) {
    if (dir1 == null || dir2 == null) return false;

    switch (dir1) {
      case "n":
        return dir2.equals("s");
      case "s":
        return dir2.equals("n");
      case "e":
        return dir2.equals("w");
      case "w":
        return dir2.equals("e");
      default:
        return false;
    }
  }

  private Set<String> getRecentOscillationDirections(String agentName) {
    Set<String> recentDirs = new HashSet<>();
    OscillationState state = agentStates.get(agentName);
    if (state != null) {
      for (MovementRecord m : state.movements) {
        recentDirs.add(m.direction);
      }
    }
    return recentDirs;
  }

  private void clearOscillationHistory(String agentName) {
    agentStates.remove(agentName);
  }

  private void recordDirection(String agentName, String direction) {
    // Keep track of last few directions to prevent immediate backtracking
    List<String> dirHistory = directionHistory.computeIfAbsent(
      agentName,
      k -> new ArrayList<>()
    );
    dirHistory.add(direction);
    if (dirHistory.size() > 5) { // Keep last 5 directions
      dirHistory.remove(0);
    }
  }

  private String getDirectionBetweenPoints(Point from, Point to) {
    if (to.x > from.x) return "e";
    if (to.x < from.x) return "w";
    if (to.y > from.y) return "s";
    if (to.y < from.y) return "n";
    return null; // Points are the same
  }

  public void updateState(String agentId, Point pos, String dir) {
    if (dir == null) return; // Don't update state if direction is null

    OscillationState state = agentStates.computeIfAbsent(
      agentId,
      k -> new OscillationState()
    );

    if (System.currentTimeMillis() - state.lastUpdateTime > PATTERN_TIMEOUT) {
      state.movements.clear();
      state.resetPatternCount();
    }

    state.addMovement(pos, dir);

    if (detectPattern(state.movements)) {
      state.patternCount++;
    } else {
      state.resetPatternCount();
    }
  }
}
