package jason.eis.movements;

import eis.iilang.*;
import jason.eis.LocalMap;
import jason.eis.Point;
import jason.eis.Zone;
import java.util.*;
import java.util.logging.Logger;

public class RandomMovement extends Movement {
  private static final int STUCK_THRESHOLD = 3;
  private static final int BOUNDARY_ESCAPE_ATTEMPTS = 5;
  private static final double RANDOM_ESCAPE_PROBABILITY = 0.4;
  private static final String FAILED_FORBIDDEN = "failed_forbidden";
  private static final int MAX_FAILURES_PER_DIRECTION = 3;

  // Movement tracking
  private final Map<String, MovementHistory> agentMovement = new HashMap<>();
  private final Map<String, Zone> agentZones = new HashMap<>();
  private final Map<String, LocalMap> agentMaps;

  // Zone constants
  private static final int ZONE_SIZE = 10;
  private static final int STEPS_BEFORE_ZONE_CHANGE = 15;
  private static final double ZONE_CHANGE_PROBABILITY = 0.3;
  private static final int MAX_ZONE_DISTANCE = 50;

  private static class MovementHistory {
    Point lastPosition;
    int stuckCount;
    String lastFailure;
    int failureCount;
    Map<String, Integer> directionFailures;
    Set<String> boundaryDirections;
    int boundaryEscapeAttempts;
    long lastUpdateTime;

    MovementHistory() {
      lastPosition = new Point(0, 0);
      stuckCount = 0;
      lastFailure = null;
      failureCount = 0;
      directionFailures = new HashMap<>();
      boundaryDirections = new HashSet<>();
      boundaryEscapeAttempts = 0;
      lastUpdateTime = System.currentTimeMillis();
    }

    void recordFailure(String direction, String failureType) {
      lastFailure = failureType;
      failureCount++;
      directionFailures.merge(direction, 1, Integer::sum);
    }

    void recordSuccess() {
      lastFailure = null;
      failureCount = 0;
      directionFailures.clear();
    }

    boolean isDirectionReliable(String direction) {
      return (
        directionFailures.getOrDefault(direction, 0) <
        MAX_FAILURES_PER_DIRECTION
      );
    }

    void recordBoundary(String direction) {
      boundaryDirections.add(direction);
      boundaryEscapeAttempts++;
    }

    void clearBoundaryAttempts() {
      boundaryEscapeAttempts = 0;
    }
  }

  private static class DirectionInfo {
    final List<String> safeTowardsTarget = new ArrayList<>(4);
    final List<String> allSafe = new ArrayList<>(4);
  }

  public RandomMovement(Map<String, LocalMap> agentMaps) {
    super(null);
    this.agentMaps = agentMaps;
  }

  @Override
  protected Point getCurrentPosition() {
    logger.warning("Called getCurrentPosition without agent name");
    return new Point(0, 0);
  }

  protected Point getCurrentPosition(String agName) {
    LocalMap map = agentMaps.get(agName);
    if (map != null) {
      return map.getCurrentPosition();
    }
    logger.warning("No LocalMap found for agent " + agName);
    return new Point(0, 0);
  }

  public String chooseBestDirection(String agName, int targetX, int targetY) {
    try {
      Point currentPos = getCurrentPosition(agName);
      MovementHistory history = agentMovement.computeIfAbsent(
        agName,
        k -> new MovementHistory()
      );

      if (currentPos.equals(history.lastPosition)) {
        if (++history.stuckCount >= STUCK_THRESHOLD) {
          String escapeDir = handleStuckAgent(agName, history.lastFailure);
          if (escapeDir != null) return escapeDir;
        }
      } else {
        history.stuckCount = 0;
        history.lastPosition = currentPos;
      }

      DirectionInfo dirInfo = analyzeDirections(agName, targetX, targetY);
      String zoneDir = handleZoneExploration(agName, dirInfo.allSafe);
      if (zoneDir != null) return zoneDir;

      if (!dirInfo.safeTowardsTarget.isEmpty()) {
        return dirInfo.safeTowardsTarget.get(0);
      }

      return getRandomSafeDirection(dirInfo.allSafe);
    } catch (Exception e) {
      logger.warning("Error in chooseBestDirection: " + e.getMessage());
      return DIRECTIONS[random.nextInt(DIRECTIONS.length)];
    }
  }

  private String handleStuckAgent(String agName, String lastFailure) {
    MovementHistory history = agentMovement.get(agName);
    if (history == null) return null;

    DirectionInfo dirInfo = analyzeDirections(agName, 0, 0);
    List<String> safeDirections = new ArrayList<>(dirInfo.allSafe);
    safeDirections.removeAll(history.boundaryDirections);

    if (history.boundaryEscapeAttempts >= BOUNDARY_ESCAPE_ATTEMPTS) {
      String escapeDir = forceBoundaryEscape(history, safeDirections);
      if (escapeDir != null) {
        history.clearBoundaryAttempts();
        return escapeDir;
      }
    }

    if (
      random.nextDouble() < RANDOM_ESCAPE_PROBABILITY &&
      !safeDirections.isEmpty()
    ) {
      return safeDirections.get(random.nextInt(safeDirections.size()));
    }

    if (FAILED_FORBIDDEN.equals(lastFailure)) {
      history.recordBoundary(getLastAttemptedDirection(agName));
      String oppositeDir = OPPOSITE_DIRECTIONS.get(
        getLastAttemptedDirection(agName)
      );
      if (safeDirections.contains(oppositeDir)) {
        return oppositeDir;
      }
    }

    if (history.stuckCount > STUCK_THRESHOLD) {
      String perpDir = getPerpendicularEscapeDirection(history, safeDirections);
      if (perpDir != null) return perpDir;
    }

    return safeDirections
      .stream()
      .filter(dir -> !history.boundaryDirections.contains(dir))
      .min(
        Comparator.comparingInt(
          dir -> history.directionFailures.getOrDefault(dir, 0)
        )
      )
      .orElse(getRandomSafeDirection(safeDirections));
  }

  private String handleZoneExploration(
    String agName,
    List<String> safeDirections
  ) {
    try {
      Point currentPos = getCurrentPosition(agName);
      Zone currentZone = getOrCreateZone(agName, currentPos);
      currentZone.recordExploration(currentPos);

      if (shouldChangeZone(currentZone, currentPos)) {
        Zone newZone = calculateNextZone(agName, currentPos);
        agentZones.put(agName, newZone);
        currentZone = newZone;
      }

      String zoneDir = getOptimalZoneDirection(
        currentPos,
        currentZone,
        safeDirections
      );
      if (zoneDir != null && safeDirections.contains(zoneDir)) {
        return zoneDir;
      }

      return safeDirections.get(random.nextInt(safeDirections.size()));
    } catch (Exception e) {
      logger.fine("Error in handleZoneExploration: " + e.getMessage());
      return safeDirections.get(random.nextInt(safeDirections.size()));
    }
  }

  private DirectionInfo analyzeDirections(
    String agName,
    int targetX,
    int targetY
  ) {
    DirectionInfo info = new DirectionInfo();
    for (String dir : DIRECTIONS) {
      if (isTowardsTarget(dir, targetX, targetY)) {
        info.safeTowardsTarget.add(dir);
      }
      info.allSafe.add(dir);
    }
    return info;
  }

  private String getRandomSafeDirection(List<String> safeDirections) {
    if (safeDirections.isEmpty()) {
      return DIRECTIONS[random.nextInt(DIRECTIONS.length)];
    }
    return safeDirections.get(random.nextInt(safeDirections.size()));
  }

  private String forceBoundaryEscape(
    MovementHistory history,
    List<String> safeDirections
  ) {
    String mainBoundary = history
      .boundaryDirections.stream()
      .max(
        Comparator.comparingInt(
          dir -> history.directionFailures.getOrDefault(dir, 0)
        )
      )
      .orElse(null);

    if (mainBoundary != null) {
      List<String> perpDirs = getPerpendicularDirections(mainBoundary);
      perpDirs.retainAll(safeDirections);
      if (!perpDirs.isEmpty()) {
        return perpDirs.get(random.nextInt(perpDirs.size()));
      }
    }
    return null;
  }

  private String getLastAttemptedDirection(String agName) {
    MovementHistory history = agentMovement.get(agName);
    return history != null ? history.lastFailure : "n";
  }

  private String getPerpendicularEscapeDirection(
    MovementHistory history,
    List<String> safeDirections
  ) {
    String lastDir = history
      .directionFailures.entrySet()
      .stream()
      .max(Map.Entry.comparingByValue())
      .map(Map.Entry::getKey)
      .orElse(null);

    if (lastDir != null) {
      List<String> perpDirs = getPerpendicularDirections(lastDir);
      perpDirs.retainAll(safeDirections);
      if (!perpDirs.isEmpty()) {
        return perpDirs.get(random.nextInt(perpDirs.size()));
      }
    }
    return null;
  }

  private Zone getOrCreateZone(String agName, Point position) {
    return agentZones.computeIfAbsent(
      agName,
      k -> new Zone(position.x / 10, position.y / 10)
    );
  }

  public void recordFailure(
    String agName,
    String direction,
    String failureType
  ) {
    MovementHistory history = agentMovement.get(agName);
    if (history != null) {
      history.recordFailure(direction, failureType);
    }
  }

  public void recordSuccess(String agName) {
    MovementHistory history = agentMovement.get(agName);
    if (history != null) {
      history.recordSuccess();
    }
  }

  private boolean shouldChangeZone(Zone zone, Point currentPos) {
    return (
      zone.getExplorationCount() > 50 || // Too many explored points
      Math.abs(currentPos.x - zone.x * 10) > 5 ||
      Math.abs(currentPos.y - zone.y * 10) > 5
    );
  }

  private Zone calculateNextZone(String agName, Point currentPos) {
    int currentZoneX = currentPos.x / 10;
    int currentZoneY = currentPos.y / 10;

    List<Zone> potentialZones = new ArrayList<>();
    for (int dx = -1; dx <= 1; dx++) {
      for (int dy = -1; dy <= 1; dy++) {
        if (dx == 0 && dy == 0) continue;
        potentialZones.add(new Zone(currentZoneX + dx, currentZoneY + dy));
      }
    }

    if (potentialZones.isEmpty()) {
      return new Zone(
        currentZoneX + random.nextInt(3) - 1,
        currentZoneY + random.nextInt(3) - 1
      );
    }

    return potentialZones.get(random.nextInt(potentialZones.size()));
  }

  private String getOptimalZoneDirection(
    Point currentPos,
    Zone zone,
    List<String> safeDirections
  ) {
    int dx = zone.x * 10 - currentPos.x;
    int dy = zone.y * 10 - currentPos.y;

    // If we're in the zone, explore within it
    if (Math.abs(dx) < 5 && Math.abs(dy) < 5) {
      return safeDirections.isEmpty()
        ? DIRECTIONS[random.nextInt(DIRECTIONS.length)]
        : safeDirections.get(random.nextInt(safeDirections.size()));
    }

    // Move towards zone center
    String preferredDir;
    if (Math.abs(dx) > Math.abs(dy)) {
      preferredDir = dx > 0 ? "e" : "w";
    } else {
      preferredDir = dy > 0 ? "s" : "n";
    }

    return safeDirections.contains(preferredDir)
      ? preferredDir
      : getRandomSafeDirection(safeDirections);
  }
}
