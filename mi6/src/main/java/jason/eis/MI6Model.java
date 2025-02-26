package jason.eis;

import eis.EnvironmentInterfaceStandard;
import eis.iilang.*;
import jason.asSyntax.*;
import jason.eis.Point;
import jason.eis.movements.PlannedMovement;
import jason.eis.movements.RandomMovement;
import jason.environment.Environment;
import java.util.*;
import java.util.concurrent.ConcurrentHashMap;
import java.util.logging.Level;
import java.util.logging.Logger;
import java.util.stream.Collectors;

public class MI6Model {
  private final Logger logger = Logger.getLogger(
    "MI6Model." + MI6Model.class.getName()
  );
  private final EnvironmentInterfaceStandard ei;
  private final Random random = new Random();
  private static final String[] DIRECTIONS = { "n", "s", "e", "w" };
  private static final int STUCK_THRESHOLD = 3;
  private static final int CLOSE_TARGET_DISTANCE = 5;

  // Pre-compute direction opposites for O(1) lookup
  private static final Map<String, String> OPPOSITE_DIRECTIONS = new HashMap<>(
    4
  ) {

    {
      put("n", "s");
      put("s", "n");
      put("e", "w");
      put("w", "e");
    }
  };

  // Movement tracking with efficient data structures
  private final Map<String, MovementHistory> agentMovement;

  // Add failure type constants
  private static final String FAILED_PATH = "failed_path";
  private static final String FAILED_FORBIDDEN = "failed_forbidden";
  private static final String FAILED_PARAMETER = "failed_parameter";
  private static final int MAX_FAILURES_PER_DIRECTION = 3;

  // Optimized zone constants
  private static final int ZONE_SIZE = 10;
  private static final int STEPS_BEFORE_ZONE_CHANGE = 15;
  private static final double ZONE_CHANGE_PROBABILITY = 0.3; // 30% chance to change zone when threshold met
  private static final int MAX_ZONE_DISTANCE = 50; // Prevent zones from getting too far from center

  private static final int BOUNDARY_ESCAPE_ATTEMPTS = 5;
  private static final double RANDOM_ESCAPE_PROBABILITY = 0.4; // 40% chance for random direction when stuck

  private static class MovementHistory {
    Point lastPosition;
    int stuckCount;
    String lastFailure;
    int failureCount;
    Map<String, Integer> directionFailures;
    Set<String> boundaryDirections; // Track known boundary directions
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

  // Track zones per agent
  private final Map<String, Zone> agentZones;

  // In MI6Model.java, add:
  private final Map<String, LocalMap> agentMaps;
  private final Map<String, Long> lastProcessedTime;
  private static final long PERCEPT_THRESHOLD = 100; // 100ms threshold

  // Add PerceptCache class definition
  private static class PerceptCache {
    final Point position;
    final BitSet obstacles;
    final long timestamp;

    PerceptCache(Point position, BitSet obstacles) {
      this.position = position;
      this.obstacles = obstacles;
      this.timestamp = System.nanoTime();
    }

    boolean isValid() {
      return System.nanoTime() - timestamp < 100_000_000; // 100ms cache
    }
  }

  private final Map<String, PerceptCache> perceptCache;

  // Store paths for each agent
  private final Map<String, List<String>> agentPaths;
  // Track if an agent is moving towards a dispenser
  private final Map<String, Boolean> movingToDispenser;

  // Add new field declarations
  private final RandomMovement randomMovement;
  private final PlannedMovement plannedMovement;

  public MI6Model(EnvironmentInterfaceStandard ei) {
    this.ei = ei;
    this.agentMaps = new ConcurrentHashMap<>();
    this.agentMovement = new ConcurrentHashMap<>();
    this.agentZones = new ConcurrentHashMap<>();
    this.perceptCache = new ConcurrentHashMap<>();
    this.agentPaths = new ConcurrentHashMap<>();
    this.movingToDispenser = new ConcurrentHashMap<>();
    this.lastProcessedTime = new ConcurrentHashMap<>();

    LocalMap.DEBUG = true;

    this.randomMovement = new RandomMovement(agentMaps);
    this.plannedMovement = new PlannedMovement(agentMaps);
  }

  public void initializeAgent(String agName) {
    if (!agentMaps.containsKey(agName)) {
      agentMaps.put(agName, new LocalMap());
      agentMovement.put(agName, new MovementHistory());
      if (LocalMap.DEBUG) {
        logger.info(String.format("[%s] Initialized new agent", agName));
      }
    }
  }

  private LocalMap getAgentMap(String agName) {
    LocalMap map = agentMaps.get(agName);
    if (map == null) {
      throw new IllegalStateException("Agent " + agName + " not initialized");
    }
    return map;
  }

  public boolean moveTowards(String agName, String direction) throws Exception {
    try {
      ei.performAction(
        agName,
        new Action("move", new Identifier(direction.toLowerCase()))
      );
      // Update the local map position after successful movement
      LocalMap map = getAgentMap(agName);
      map.updatePositionFromMovement(direction);
      randomMovement.recordSuccess(agName);
      logMapState(agName); // Log state after successful move
      return true;
    } catch (Exception e) {
      return handleMoveFailure(agName, direction, e);
    }
  }

  private Point calculateNewPosition(Point current, String direction) {
    switch (direction.toLowerCase()) {
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

  private boolean handleMoveFailure(
    String agName,
    String direction,
    Exception e
  )
    throws Exception {
    String errorMsg = e != null ? e.getMessage() : "Unknown error";
    logger.warning("Move failed for agent " + agName + ": " + errorMsg);

    // If we have a specific error message, we might want to handle it differently
    if (errorMsg != null && errorMsg.toLowerCase().contains("forbidden")) {
      // Handle forbidden move
      randomMovement.recordFailure(agName, direction, "failed_forbidden");
      logMapState(agName); // Log state after failed move
      return false;
    }

    // Re-throw the exception if we can't handle it
    throw new Exception("Move failed: " + errorMsg);
  }

  public boolean requestBlock(String agName, String direction) {
    try {
      ei.performAction(
        agName,
        new Action("request", new Identifier(direction))
      );
      return true;
    } catch (Exception e) {
      logger.log(Level.SEVERE, "Error in requestBlock", e);
      return false;
    }
  }

  public boolean attachBlock(String agName, String direction) {
    try {
      ei.performAction(agName, new Action("attach", new Identifier(direction)));
      return true;
    } catch (Exception e) {
      logger.log(Level.SEVERE, "Error in attachBlock", e);
      return false;
    }
  }

  public String chooseBestDirection(String agName, int targetX, int targetY) {
    if (plannedMovement.isMovingToDispenser(agName)) {
      return null;
    }
    return randomMovement.chooseBestDirection(agName, targetX, targetY);
  }

  private static class DirectionInfo {
    final List<String> safeTowardsTarget = new ArrayList<>(4);
    final List<String> allSafe = new ArrayList<>(4);
  }

  private DirectionInfo analyzeDirections(
    String agName,
    int targetX,
    int targetY
  )
    throws Exception {
    DirectionInfo info = new DirectionInfo();
    Map<String, Boolean> obstacles = getImmediateObstacles(agName);

    // O(1) operation as we only check 4 directions
    for (String dir : DIRECTIONS) {
      if (!obstacles.getOrDefault(dir, false)) {
        info.allSafe.add(dir);
        if (isTowardsTarget(dir, targetX, targetY)) {
          info.safeTowardsTarget.add(dir);
        }
      }
    }

    return info;
  }

  private Map<String, Boolean> getImmediateObstacles(String agName)
    throws Exception {
    Map<String, Boolean> obstacles = new HashMap<>(4);
    Map<String, Collection<Percept>> percepts = ei.getAllPercepts(agName);

    for (Collection<Percept> perceptList : percepts.values()) {
      for (Percept p : perceptList) {
        if (p.getName().equals("thing")) {
          Parameter[] params = p.getParameters().toArray(new Parameter[0]);
          int x = ((Numeral) params[0]).getValue().intValue();
          int y = ((Numeral) params[1]).getValue().intValue();
          String type = ((Identifier) params[2]).getValue();

          if (type.equals("obstacle") || type.equals("entity")) {
            if (x == 0 && y == -1) obstacles.put("n", true);
            if (x == 0 && y == 1) obstacles.put("s", true);
            if (x == 1 && y == 0) obstacles.put("e", true);
            if (x == -1 && y == 0) obstacles.put("w", true);
          }
        }
      }
    }
    return obstacles;
  }

  private boolean isTowardsTarget(String direction, int targetX, int targetY) {
    return (
      (direction.equals("e") && targetX > 0) ||
      (direction.equals("w") && targetX < 0) ||
      (direction.equals("s") && targetY > 0) ||
      (direction.equals("n") && targetY < 0)
    );
  }

  private String getBestSafeDirection(
    List<String> safeDirections,
    int targetX,
    int targetY
  ) {
    // Prefer directions that at least move on correct axis
    for (String dir : safeDirections) {
      if (
        (
          Math.abs(targetX) > Math.abs(targetY) &&
          (dir.equals("e") && targetX > 0 || dir.equals("w") && targetX < 0)
        ) ||
        (
          Math.abs(targetY) >= Math.abs(targetX) &&
          (dir.equals("s") && targetY > 0 || dir.equals("n") && targetY < 0)
        )
      ) {
        return dir;
      }
    }
    return safeDirections.get(random.nextInt(safeDirections.size()));
  }

  private boolean isAlignedAndClear(String agName, int targetX, int targetY)
    throws Exception {
    if ((targetX == 0 || targetY == 0) && (targetX != 0 || targetY != 0)) {
      String dir = getAlignedDirection(targetX, targetY);
      Map<String, Boolean> obstacles = getImmediateObstacles(agName);
      return !obstacles.getOrDefault(dir, false);
    }
    return false;
  }

  private String getAlignedDirection(int targetX, int targetY) {
    if (targetX == 0) {
      return targetY < 0 ? "n" : "s";
    }
    return targetX < 0 ? "w" : "e";
  }

  private Point getCurrentPosition(String agName) {
    try {
      Map<String, Collection<Percept>> percepts = ei.getAllPercepts(agName);
      if (percepts != null) {
        for (Collection<Percept> perceptList : percepts.values()) {
          for (Percept p : perceptList) {
            if (p.getName().equals("position")) {
              Parameter[] params = p.getParameters().toArray(new Parameter[0]);
              int x = ((Numeral) params[0]).getValue().intValue();
              int y = ((Numeral) params[1]).getValue().intValue();
              return new Point(x, y);
            }
          }
        }
      }

      // If we can't get position, use last known position or default
      MovementHistory history = agentMovement.get(agName);
      if (history != null && history.lastPosition != null) {
        return history.lastPosition;
      }

      // If all else fails, return origin
      return new Point(0, 0);
    } catch (Exception e) {
      logger.fine(
        "Could not get position for agent " + agName + ", using default"
      );
      return new Point(0, 0);
    }
  }

  private String handleStuckAgent(String agName, String lastFailure) {
    try {
      MovementHistory history = agentMovement.get(agName);
      if (history == null) return null;

      // Get current safe directions
      DirectionInfo dirInfo = analyzeDirections(agName, 0, 0);
      List<String> safeDirections = new ArrayList<>(dirInfo.allSafe);

      // Remove known boundary directions
      safeDirections.removeAll(history.boundaryDirections);

      // If we've been trying to escape boundary too many times, force a perpendicular move
      if (history.boundaryEscapeAttempts >= BOUNDARY_ESCAPE_ATTEMPTS) {
        String escapeDir = forceBoundaryEscape(history, safeDirections);
        if (escapeDir != null) {
          history.clearBoundaryAttempts();
          return escapeDir;
        }
      }

      // Sometimes choose random direction to break patterns
      if (random.nextDouble() < RANDOM_ESCAPE_PROBABILITY) {
        if (!safeDirections.isEmpty()) {
          return safeDirections.get(random.nextInt(safeDirections.size()));
        }
      }

      // Handle specific failure types
      if (FAILED_FORBIDDEN.equals(lastFailure)) {
        history.recordBoundary(getLastAttemptedDirection(agName));
        String oppositeDir = OPPOSITE_DIRECTIONS.get(
          getLastAttemptedDirection(agName)
        );
        if (safeDirections.contains(oppositeDir)) {
          return oppositeDir;
        }
      }

      // If still stuck, try perpendicular movement
      if (history.stuckCount > STUCK_THRESHOLD) {
        String perpDir = getPerpendicularEscapeDirection(
          history,
          safeDirections
        );
        if (perpDir != null) return perpDir;
      }

      // Choose least failed direction that's not a boundary
      return safeDirections
        .stream()
        .filter(dir -> !history.boundaryDirections.contains(dir))
        .min(
          Comparator.comparingInt(
            dir -> history.directionFailures.getOrDefault(dir, 0)
          )
        )
        .orElse(getRandomSafeDirection(safeDirections));
    } catch (Exception e) {
      logger.log(Level.SEVERE, "Error in handleStuckAgent", e);
      return null;
    }
  }

  private String forceBoundaryEscape(
    MovementHistory history,
    List<String> safeDirections
  ) {
    // Get the main boundary direction (most failed)
    String mainBoundary = history
      .boundaryDirections.stream()
      .max(
        Comparator.comparingInt(
          dir -> history.directionFailures.getOrDefault(dir, 0)
        )
      )
      .orElse(null);

    if (mainBoundary != null) {
      // Get perpendicular directions
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
    // Get the most recent failed direction
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

  private String getRandomSafeDirection(List<String> safeDirections) {
    if (safeDirections.isEmpty()) {
      return DIRECTIONS[random.nextInt(DIRECTIONS.length)];
    }
    return safeDirections.get(random.nextInt(safeDirections.size()));
  }

  private String getLastAttemptedDirection(String agName) {
    // Implement the logic to get the last attempted direction
    // This is a placeholder and should be replaced with the actual implementation
    return "n"; // Placeholder return, actual implementation needed
  }

  private String handleZoneExploration(
    String agName,
    List<String> safeDirections
  ) {
    try {
      Point currentPos = getCurrentPosition(agName);
      Zone currentZone = agentZones.computeIfAbsent(
        agName,
        k -> new Zone(0, 0)
      );
      currentZone.recordExploration(currentPos);

      // Check if we should change zones
      boolean shouldChangeZone = shouldChangeZone(currentZone, currentPos);

      if (shouldChangeZone) {
        Zone newZone = calculateNextZone(agName, currentPos);
        agentZones.put(agName, newZone);
        currentZone = newZone;
      }

      // Get direction towards zone center
      String zoneDir = getOptimalZoneDirection(
        currentPos,
        currentZone,
        safeDirections
      );
      if (zoneDir != null && safeDirections.contains(zoneDir)) {
        return zoneDir;
      }

      // If we can't move towards zone center, use safe random direction
      return safeDirections.get(random.nextInt(safeDirections.size()));
    } catch (Exception e) {
      logger.log(Level.FINE, "Error in handleZoneExploration", e);
      return safeDirections.get(random.nextInt(safeDirections.size()));
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
    int dx = zone.x - currentPos.x;
    int dy = zone.y - currentPos.y;

    // If we're in the zone, explore within it
    if (Math.abs(dx) < ZONE_SIZE / 2 && Math.abs(dy) < ZONE_SIZE / 2) {
      return exploreWithinZone(zone, safeDirections);
    }

    // Move towards zone center
    if (Math.abs(dx) > Math.abs(dy)) {
      return dx > 0 ? "e" : "w";
    } else {
      return dy > 0 ? "s" : "n";
    }
  }

  private String exploreWithinZone(Zone zone, List<String> safeDirections) {
    if (safeDirections.isEmpty()) {
      return DIRECTIONS[random.nextInt(DIRECTIONS.length)];
    }
    return safeDirections.get(random.nextInt(safeDirections.size()));
  }

  private Set<Point> getExploredPoints(String agName) {
    // Implementation for getting explored points
    return new HashSet<>(); // Placeholder
  }

  private List<String> getPerpendicularDirections(String direction) {
    List<String> perpDirs = new ArrayList<>(2);
    if ("n".equals(direction) || "s".equals(direction)) {
      perpDirs.add("e");
      perpDirs.add("w");
    } else {
      perpDirs.add("n");
      perpDirs.add("s");
    }
    return perpDirs;
  }

  public void processPercepts(String agName, Collection<Percept> percepts) {
    try {
      long currentTime = System.currentTimeMillis();
      Long lastTime = lastProcessedTime.get(agName);

      // Skip if we processed percepts too recently
      if (lastTime != null && currentTime - lastTime < PERCEPT_THRESHOLD) {
        return;
      }

      if (!agentMaps.containsKey(agName)) {
        initializeAgent(agName);
      }

      LocalMap map = getAgentMap(agName);

      // Process all percepts
      for (Percept p : percepts) {
        String name = p.getName();
        Parameter[] params = p.getParameters().toArray(new Parameter[0]);

        switch (name) {
          case "thing":
            processThingPercept(agName, p);
            break;
          case "obstacle":
            int obstX = ((Numeral) params[0]).getValue().intValue();
            int obstY = ((Numeral) params[1]).getValue().intValue();
            map.addObstacle(new Point(obstX, obstY));
            break;
          case "goal":
            int goalX = ((Numeral) params[0]).getValue().intValue();
            int goalY = ((Numeral) params[1]).getValue().intValue();
            map.addGoal(new Point(goalX, goalY));
            break;
        }
      }

      map.clearStaleEntities();
      lastProcessedTime.put(agName, currentTime);
    } catch (Exception e) {
      logger.log(Level.WARNING, "Error processing percepts for " + agName, e);
    }
  }

  private void processThingPercept(String agName, Percept p) {
    try {
      Parameter[] params = p.getParameters().toArray(new Parameter[0]);
      int relX = ((Numeral) params[0]).getValue().intValue();
      int relY = ((Numeral) params[1]).getValue().intValue();
      String type = ((Identifier) params[2]).getValue();
      String details = params.length > 3
        ? ((Identifier) params[3]).getValue()
        : null;

      LocalMap map = getAgentMap(agName);
      Point relativePos = new Point(relX, relY);

      if (LocalMap.DEBUG) {
        logger.info(
          String.format(
            "[%s] Processing thing: %s at (%d,%d) [abs: %s] with details %s",
            agName,
            type,
            relX,
            relY,
            map.getCurrentPosition(),
            details
          )
        );
      }

      switch (type) {
        case "dispenser":
          if (
            details != null && (details.equals("b0") || details.equals("b1"))
          ) {
            map.addDispenser(relativePos, details);
          }
          break;
        case "block":
          if (
            details != null && (details.equals("b0") || details.equals("b1"))
          ) {
            map.addBlock(relativePos, details);
          }
          break;
      }
    } catch (Exception e) {
      logger.warning("Error processing thing percept: " + e.getMessage());
    }
  }

  private Zone getOrCreateZone(String agName, Point position) {
    return agentZones.computeIfAbsent(
      agName,
      k -> new Zone(position.x / 10, position.y / 10)
    );
  }

  public void updateExploration(String agName, Point currentPos) {
    Zone currentZone = getOrCreateZone(agName, currentPos);
    currentZone.recordExploration(currentPos);

    if (shouldChangeZone(currentZone, currentPos)) {
      Zone newZone = calculateNextZone(agName, currentPos);
      agentZones.put(agName, newZone);
    }
  }

  public boolean findNearestDispenser(String agName) {
    return plannedMovement.findNearestDispenser(agName);
  }

  public boolean moveToNearestDispenser(String agName) {
    String nextMove = plannedMovement.getNextMove(agName);
    if (nextMove == null) return false;

    try {
      boolean success = moveTowards(agName, nextMove);
      if (success) {
        plannedMovement.moveSucceeded(agName);
      } else {
        plannedMovement.moveFailed(agName);
      }
      return success;
    } catch (Exception e) {
      logger.log(Level.SEVERE, "Error moving agent " + agName, e);
      plannedMovement.moveFailed(agName);
      return false;
    }
  }

  public List<String> getPathToNearestDispenser(String agName) {
    return agentPaths.getOrDefault(agName, Collections.emptyList());
  }

  public void logMapState(String agName) {
    if (LocalMap.DEBUG) {
      try {
        LocalMap map = getAgentMap(agName);
        logger.info(
          "\n=== Map State for Agent " +
          agName +
          " ===\n" +
          map.toString() +
          "\n================================"
        );
      } catch (Exception e) {
        logger.warning(
          "Failed to log map state for agent " + agName + ": " + e.getMessage()
        );
      }
    }
  }
}
