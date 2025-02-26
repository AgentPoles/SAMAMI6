package jason.eis.movements;

import eis.iilang.*;
import jason.eis.LocalMap;
import jason.eis.Point;
import jason.eis.Zone;
import java.util.*;
import java.util.BitSet;
import java.util.concurrent.ConcurrentHashMap;
import java.util.logging.Logger;
import java.util.stream.Collectors;

public class RandomMovement extends Movement {
  private static final int STUCK_THRESHOLD = 3;
  private static final int BOUNDARY_ESCAPE_ATTEMPTS = 5;
  private static final double RANDOM_ESCAPE_PROBABILITY = 0.4;
  private static final String FAILED_FORBIDDEN = "failed_forbidden";
  private static final int MAX_FAILURES_PER_DIRECTION = 3;
  private static final int DIRECTION_CACHE_SIZE = 10;

  // Core constants for movement optimization
  private static final String[] DIRECTIONS = { "n", "s", "e", "w" };
  private static final Map<String, Integer> DIRECTION_INDEX = new HashMap<>(4) {

    {
      put("n", 0);
      put("s", 1);
      put("e", 2);
      put("w", 3);
    }
  };

  // Movement tracking with efficient data structures
  private final Map<String, MovementState> agentStates;
  private final DirectionCache directionCache;

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
    super(agentMaps);
    this.agentMaps = agentMaps;
    this.agentStates = new ConcurrentHashMap<>();
    this.directionCache = new DirectionCache(DIRECTION_CACHE_SIZE);
  }

  // Efficient state tracking per agent
  private static class MovementState {
    Point position;
    int stuckCount;
    byte[] directionFailures; // Using byte array instead of Map
    BitSet boundaryDirections;
    int escapeAttempts;
    String lastFailedDirection;
    long lastUpdateTime;

    MovementState() {
      position = new Point(0, 0);
      stuckCount = 0;
      directionFailures = new byte[4]; // One for each direction
      boundaryDirections = new BitSet(4);
      escapeAttempts = 0;
      lastUpdateTime = System.nanoTime();
    }

    void recordFailure(String direction) {
      int dirIndex = DIRECTION_INDEX.get(direction);
      if (directionFailures[dirIndex] < Byte.MAX_VALUE) {
        directionFailures[dirIndex]++;
      }
      lastFailedDirection = direction;
    }

    void recordSuccess() {
      Arrays.fill(directionFailures, (byte) 0);
      lastFailedDirection = null;
    }

    boolean isDirectionSafe(String direction) {
      return (
        directionFailures[DIRECTION_INDEX.get(direction)] <
        MAX_FAILURES_PER_DIRECTION
      );
    }
  }

  // Cache recent successful directions
  private static class DirectionCache {
    private final String[] directions;
    private final Point[] positions;
    private int index;
    private final int size;

    DirectionCache(int size) {
      this.size = size;
      this.directions = new String[size];
      this.positions = new Point[size];
      this.index = 0;
    }

    void add(String direction, Point position) {
      directions[index] = direction;
      positions[index] = position;
      index = (index + 1) % size;
    }

    String getBestDirection(Point currentPos) {
      for (int i = 0; i < size; i++) {
        if (
          positions[i] != null &&
          (
            Math.abs(positions[i].x - currentPos.x) +
            Math.abs(positions[i].y - currentPos.y)
          ) <
          5
        ) {
          return directions[i];
        }
      }
      return null;
    }
  }

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
      MovementState state = getAgentState(agName);
      Point currentPos = getCurrentPosition(agName);

      // Check cache first for quick decision
      String cachedDirection = directionCache.getBestDirection(currentPos);
      if (cachedDirection != null && isValidMove(agName, cachedDirection)) {
        return cachedDirection;
      }

      // Handle stuck detection
      if (currentPos.equals(state.position)) {
        if (++state.stuckCount >= STUCK_THRESHOLD) {
          return handleStuckAgent(agName, state);
        }
      } else {
        state.position = currentPos;
        state.stuckCount = 0;
      }

      // Get optimal direction based on target
      return calculateOptimalDirection(agName, state, targetX, targetY);
    } catch (Exception e) {
      logger.warning("Error choosing direction: " + e.getMessage());
      return DIRECTIONS[random.nextInt(DIRECTIONS.length)];
    }
  }

  private String handleStuckAgent(String agName, MovementState state) {
    // Try perpendicular direction first
    if (state.lastFailedDirection != null) {
      List<String> perpDirs = getPerpendicularDirections(
        state.lastFailedDirection
      );
      for (String dir : perpDirs) {
        if (isValidMove(agName, dir)) {
          return dir;
        }
      }
    }

    // Random escape attempt
    if (random.nextDouble() < RANDOM_ESCAPE_PROBABILITY) {
      String[] safeDirs = Arrays
        .stream(DIRECTIONS)
        .filter(dir -> isValidMove(agName, dir))
        .toArray(String[]::new);
      if (safeDirs.length > 0) {
        return safeDirs[random.nextInt(safeDirs.length)];
      }
    }

    // Default to least failed direction
    return Arrays
      .stream(DIRECTIONS)
      .filter(dir -> isValidMove(agName, dir))
      .min(
        Comparator.comparingInt(
          dir -> state.directionFailures[DIRECTION_INDEX.get(dir)]
        )
      )
      .orElse(DIRECTIONS[random.nextInt(DIRECTIONS.length)]);
  }

  private String calculateOptimalDirection(
    String agName,
    MovementState state,
    int targetX,
    int targetY
  ) {
    Point currentPos = getCurrentPosition(agName);
    List<String> validDirs = Arrays
      .stream(DIRECTIONS)
      .filter(dir -> isValidMove(agName, dir))
      .collect(Collectors.toList());

    if (validDirs.isEmpty()) {
      return DIRECTIONS[random.nextInt(DIRECTIONS.length)];
    }

    // Prefer directions towards target
    return validDirs
      .stream()
      .filter(
        dir ->
          isTowardsTarget(dir, targetX - currentPos.x, targetY - currentPos.y)
      )
      .findFirst()
      .orElse(validDirs.get(random.nextInt(validDirs.size())));
  }

  private String getLastAttemptedDirection(String agName) {
    MovementState state = getAgentState(agName);
    return state.lastFailedDirection != null
      ? state.lastFailedDirection
      : DIRECTIONS[0];
  }

  private MovementState getAgentState(String agName) {
    return agentStates.computeIfAbsent(agName, k -> new MovementState());
  }

  protected boolean isValidMove(String agName, String direction) {
    MovementState state = getAgentState(agName);
    return (
      state.isDirectionSafe(direction) &&
      !state.boundaryDirections.get(DIRECTION_INDEX.get(direction))
    );
  }

  public void recordSuccess(String agName) {
    MovementState state = getAgentState(agName);
    state.recordSuccess();
    Point currentPos = getCurrentPosition(agName);
    directionCache.add(getLastAttemptedDirection(agName), currentPos);
  }

  public void recordFailure(
    String agName,
    String direction,
    String failureType
  ) {
    MovementState state = getAgentState(agName);
    state.recordFailure(direction);
    if (FAILED_FORBIDDEN.equals(failureType)) {
      state.boundaryDirections.set(DIRECTION_INDEX.get(direction));
    }
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
