package jason.eis;

import jason.asSyntax.Atom;
import jason.asSyntax.NumberTerm;
import jason.asSyntax.Term;
import jason.eis.movements.Search;
import java.util.*;
import java.util.concurrent.ConcurrentHashMap;
import java.util.logging.Logger;
import java.util.stream.Collectors;

public class LocalMap {
  private static final Logger logger = Logger.getLogger(
    LocalMap.class.getName()
  );
  private static final int CELL_SIZE = 8; // Power of 2 for efficient division
  private static final int STALE_THRESHOLD = 30000; // 30 seconds in milliseconds
  public static boolean DEBUG = false;

  // Current position tracking
  private Point currentPosition;

  // Efficient spatial indexing
  private final SpatialGrid spatialGrid;

  // Type-based indexing for quick entity type access
  private final Map<EntityType, Map<String, Entity>> typeIndex;

  // Entity registry for quick entity lookup by ID
  private final Map<String, Entity> entityRegistry;

  private final Set<Point> obstacles;

  // Add a lock for position updates
  private final Object positionLock = new Object();

  // Add debug tracking map
  private final Map<String, EntityDebugInfo> debugTrackingMap;

  private final Map<Point, ObstacleInfo> dynamicObstacles = new ConcurrentHashMap<>();
  private static final int DYNAMIC_OBSTACLE_TTL = 3000; // 3 seconds time-to-live
  public static final int CRITICAL_DISTANCE = 2; // Distance where obstacles become critical to avoid
  public static final int AWARENESS_DISTANCE = 5; // Max distance to track obstacles

  private Point mapMinBounds = null;
  private Point mapMaxBounds = null;
  private boolean boundsInitialized = false;

  // Boundary detection constants
  private static final int MIN_BOUNDARY_CONFIRMATIONS = 3;
  private final Map<String, Integer> boundaryConfirmations = new HashMap<>(); // Tracks how many times we've hit each boundary

  private final Map<String, BoundaryEvidence> boundaryEvidence = new HashMap<>();
  private static final int EVIDENCE_THRESHOLD = 3; // Number of confirmations needed

  private final Map<String, BoundaryInfo> confirmedBoundaries = new HashMap<>();
  private final Map<Point, ObstacleInfo> staticObstacles = new HashMap<>();
  private final Map<Point, Integer> temporaryObstacles = new HashMap<>();
  // Add these collections if they don't exist
  private final Map<Point, Entity> dispensers = new HashMap<>();
  private final Map<Point, Entity> blocks = new HashMap<>();
  private final Map<Point, Entity> goals = new HashMap<>();

  // Add constants for boundary and obstacle handling
  private static final int BOUNDARY_CONFIRMATION_THRESHOLD = 2;
  private static final int TEMP_OBSTACLE_TTL = 5; // Time-to-live in steps

  // Add tracking maps
  private Point lastAttemptedMove = null;

  public enum EntityType {
    DISPENSER,
    BLOCK,
    GOAL,
    AGENT,
    OBSTACLE,
  }

  public static class Entity {
    private final String id;
    private final EntityType type;
    private final String details;
    private final Point position;
    private final Point relativePos;
    private long lastSeen;

    public Entity(
      String id,
      EntityType type,
      String details,
      Point position,
      Point relativePos
    ) {
      this.id = id;
      this.type = type;
      this.details = details;
      this.position = position;
      this.relativePos = relativePos;
      this.lastSeen = System.currentTimeMillis();
    }

    public String getId() {
      return id;
    }

    public EntityType getType() {
      return type;
    }

    public String getDetails() {
      return details;
    }

    public Point getPosition() {
      return position;
    }

    public Point getRelativePos() {
      return relativePos;
    }

    public void updateLastSeen() {
      this.lastSeen = System.currentTimeMillis();
    }

    public boolean isStale() {
      return System.currentTimeMillis() - lastSeen > STALE_THRESHOLD;
    }
  }

  private static class SpatialGrid {
    private final Map<Point, Set<String>> grid = new ConcurrentHashMap<>();

    void add(Point p, String entityId) {
      grid.computeIfAbsent(p, k -> ConcurrentHashMap.newKeySet()).add(entityId);
    }

    void remove(Point p, String entityId) {
      Set<String> cell = grid.get(p);
      if (cell != null) {
        cell.remove(entityId);
        if (cell.isEmpty()) {
          grid.remove(p);
        }
      }
    }

    Set<String> getEntitiesInRange(Point center, int range) {
      Set<String> result = ConcurrentHashMap.newKeySet();

      // Use exact coordinate matching within range
      for (int x = -range; x <= range; x++) {
        for (int y = -range; y <= range; y++) {
          Point p = new Point(center.x + x, center.y + y);
          Set<String> entities = grid.get(p);
          if (entities != null) {
            result.addAll(entities);
          }
        }
      }
      return result;
    }

    void move(Point oldPos, Point newPos, String entityId) {
      remove(oldPos, entityId);
      add(newPos, entityId);
    }

    public Collection<Entity> getNearbyEntities(int x, int y, int range) {
      List<Entity> nearby = new ArrayList<>();
      for (int dx = -range; dx <= range; dx++) {
        for (int dy = -range; dy <= range; dy++) {
          int checkX = x + dx;
          int checkY = y + dy;
          Entity entity = getEntity(checkX, checkY);
          if (entity != null) {
            nearby.add(entity);
          }
        }
      }
      return nearby;
    }

    private Entity getEntity(int x, int y) {
      // Implement based on your grid structure
      // This is a placeholder implementation
      return null;
    }
  }

  // Debug tracking class
  private static class EntityDebugInfo {
    final Point relativePos;
    final Point agentAbsPos;
    final long timestamp;

    EntityDebugInfo(Point relativePos, Point agentAbsPos) {
      this.relativePos = relativePos;
      this.agentAbsPos = agentAbsPos;
      this.timestamp = System.currentTimeMillis();
    }

    @Override
    public String toString() {
      return String.format(
        "relative(%d,%d) from agent(%d,%d)",
        relativePos.x,
        relativePos.y,
        agentAbsPos.x,
        agentAbsPos.y
      );
    }
  }

  private static class BoundaryEvidence {
    int failedMoves = 0;
    int continuousObstacles = 0;
    Point lastConfirmedPosition = null;

    void addFailedMove(Point position) {
      failedMoves++;
      lastConfirmedPosition = position;
    }

    void addObstacleEvidence(int count) {
      continuousObstacles = Math.max(continuousObstacles, count);
    }

    boolean isBoundaryConfirmed() {
      // Confirm boundary if either:
      // 1. Multiple failed moves in same area
      // 2. Long continuous line of obstacles at vision limit
      return (
        failedMoves >= EVIDENCE_THRESHOLD ||
        continuousObstacles >= EVIDENCE_THRESHOLD
      );
    }
  }

  private static class BoundaryInfo {
    Point position;
    long confirmationTime;

    BoundaryInfo(Point pos) {
      this.position = pos;
      this.confirmationTime = System.currentTimeMillis();
    }
  }

  public static class ObstacleInfo {
    private final Point position;
    private Point velocity; // Add velocity tracking
    private long lastSeen;
    private final String type;
    private final boolean isDynamic;
    private Point lastPosition; // Add last position tracking
    private boolean hasBlock = false;

    public ObstacleInfo(Point pos, String type, boolean isDynamic) {
      this.position = pos;
      this.type = type;
      this.isDynamic = isDynamic;
      this.lastSeen = System.currentTimeMillis();
      this.velocity = null;
      this.lastPosition = null;
    }

    public void updateSeen() {
      if (lastPosition != null && !position.equals(lastPosition)) {
        // Calculate velocity based on position change
        velocity =
          new Point(position.x - lastPosition.x, position.y - lastPosition.y);
      }
      lastPosition = position;
      this.lastSeen = System.currentTimeMillis();
    }

    public Point predictPosition(int steps) {
      if (!isDynamic || velocity == null) return position;

      return new Point(
        position.x + (velocity.x * steps),
        position.y + (velocity.y * steps)
      );
    }

    public boolean isStale() {
      return isDynamic && System.currentTimeMillis() - lastSeen > 5000; // 5 seconds
    }

    public Point getPosition() {
      return position;
    }

    public String getType() {
      return type;
    }

    public boolean isDynamic() {
      return isDynamic;
    }

    public boolean hasBlock() {
      return hasBlock;
    }

    public void setHasBlock(boolean hasBlock) {
      this.hasBlock = hasBlock;
    }
  }

  public LocalMap() {
    this.obstacles = new HashSet<>();
    this.currentPosition = new Point(0, 0);
    this.spatialGrid = new SpatialGrid();
    this.typeIndex = new EnumMap<>(EntityType.class);
    this.entityRegistry = new ConcurrentHashMap<>();
    this.debugTrackingMap = DEBUG ? new ConcurrentHashMap<>() : null;

    // Initialize type index
    for (EntityType type : EntityType.values()) {
      typeIndex.put(type, new ConcurrentHashMap<>());
    }
  }

  // Position management
  public Point getCurrentPosition() {
    return new Point(currentPosition.x, currentPosition.y);
  }

  public void updatePosition(Point newPosition) {
    synchronized (positionLock) {
      if (DEBUG) {
        logger.fine(
          String.format(
            "Updating position from (%d,%d) to (%d,%d)",
            currentPosition.x,
            currentPosition.y,
            newPosition.x,
            newPosition.y
          )
        );
      }
      currentPosition = newPosition;
    }
  }

  public void updatePositionFromMovement(String direction) {
    synchronized (positionLock) {
      Point newPosition;
      switch (direction.toLowerCase()) {
        case "n":
          newPosition = new Point(currentPosition.x, currentPosition.y - 1);
          break;
        case "s":
          newPosition = new Point(currentPosition.x, currentPosition.y + 1);
          break;
        case "e":
          newPosition = new Point(currentPosition.x + 1, currentPosition.y);
          break;
        case "w":
          newPosition = new Point(currentPosition.x - 1, currentPosition.y);
          break;
        default:
          return;
      }

      if (DEBUG) {
        logger.info(
          String.format(
            "Moving %s: Updating position from (%d,%d) to (%d,%d)",
            direction,
            currentPosition.x,
            currentPosition.y,
            newPosition.x,
            newPosition.y
          )
        );
      }
      currentPosition = newPosition;
    }
  }

  // Entity management
  private void addEntity(
    EntityType type,
    String subType,
    Point relativePos,
    Point currentAbsPos
  ) {
    synchronized (positionLock) {
      // Calculate absolute position using the provided current absolute position
      Point absolutePos = new Point(
        currentAbsPos.x + relativePos.x,
        currentAbsPos.y + relativePos.y
      );

      // First, clean up any stale entities
      cleanupStaleEntities(type);

      // Check if this exact entity already exists
      String entityId = generateEntityId(type, absolutePos, subType);
      Entity existingEntity = entityRegistry.get(entityId);

      if (existingEntity != null && !existingEntity.isStale()) {
        // Just update the last seen time
        existingEntity.updateLastSeen();
        return;
      }

      Entity entity = new Entity(
        entityId,
        type,
        subType,
        absolutePos,
        currentAbsPos
      );

      entityRegistry.put(entityId, entity);
      typeIndex.get(type).put(entityId, entity);
      spatialGrid.add(absolutePos, entityId);

      // Add debug tracking
      if (DEBUG) {
        debugTrackingMap.put(
          entityId,
          new EntityDebugInfo(relativePos, currentAbsPos)
        );
        logger.info(
          String.format(
            "[%s] Adding entity at abs(%d,%d) calculated from %s",
            type,
            absolutePos.x,
            absolutePos.y,
            debugTrackingMap.get(entityId)
          )
        );
      }
    }
  }

  private void cleanupStaleEntities(EntityType type) {
    // Commenting out stale entity cleanup for now to maintain full history
    /*
    long now = System.currentTimeMillis();
    
    Set<String> staleEntities = typeIndex.get(type).values().stream()
        .filter(e -> now - e.lastSeen > STALE_THRESHOLD && !e.position.equals(currentPosition))
        .map(e -> e.id)
        .collect(Collectors.toSet());

    for (String entityId : staleEntities) {
        Entity entity = entityRegistry.get(entityId);
        if (entity != null) {
            entityRegistry.remove(entityId);
            typeIndex.get(entity.type).remove(entityId);
            spatialGrid.remove(entity.position, entityId);
            
            if (DEBUG) {
                debugTrackingMap.remove(entityId);
            }
        }
    }
    */
  }

  private String generateEntityId(EntityType type, Point pos, String subType) {
    return String.format(
      "%s_%d_%d_%s",
      type,
      pos.x,
      pos.y,
      subType != null ? subType : ""
    );
  }

  // Public API for adding entities
  public void addDispenser(
    Point relativePos,
    String details,
    Point currentAbsPos
  ) {
    addEntity(EntityType.DISPENSER, details, relativePos, currentAbsPos);

    // Also maintain the legacy dispensers map for backward compatibility
    Point absolutePos = new Point(
      currentAbsPos.x + relativePos.x,
      currentAbsPos.y + relativePos.y
    );
    dispensers.put(
      absolutePos,
      entityRegistry.get(
        generateEntityId(EntityType.DISPENSER, absolutePos, details)
      )
    );
  }

  public void addBlock(Point relativePos, String details, Point currentAbsPos) {
    Point absolutePos = new Point(
      currentAbsPos.x + relativePos.x,
      currentAbsPos.y + relativePos.y
    );
    blocks.put(
      absolutePos,
      new Entity(
        "block_" + absolutePos.toString(),
        EntityType.BLOCK,
        details,
        absolutePos,
        relativePos
      )
    );
  }

  public void addObstacle(Point relativePos, Point currentAbsPos) {
    synchronized (positionLock) {
      Point absolutePos = new Point(
        currentAbsPos.x + relativePos.x,
        currentAbsPos.y + relativePos.y
      );

      // Add as an entity
      addEntity(EntityType.OBSTACLE, "static", relativePos, currentAbsPos);

      // Maintain legacy collections for backward compatibility
      obstacles.add(absolutePos);
      staticObstacles.putIfAbsent(
        absolutePos,
        new ObstacleInfo(absolutePos, "static", false)
      );
    }
  }

  public void addGoal(Point relativePos, Point currentAbsPos) {
    addEntity(EntityType.GOAL, "goal", relativePos, currentAbsPos);

    // Also maintain the legacy goals map for backward compatibility
    Point absolutePos = new Point(
      currentAbsPos.x + relativePos.x,
      currentAbsPos.y + relativePos.y
    );
    goals.put(
      absolutePos,
      entityRegistry.get(generateEntityId(EntityType.GOAL, absolutePos, "goal"))
    );
  }

  // Query methods for external search algorithms
  public Set<Point> getEntitiesInRange(
    Point center,
    int range,
    EntityType type
  ) {
    // Ensure center is in absolute coordinates
    Point absoluteCenter = new Point(
      currentPosition.x + center.x,
      currentPosition.y + center.y
    );

    Set<Point> result = new HashSet<>();
    Set<String> entitiesInRange = spatialGrid.getEntitiesInRange(
      absoluteCenter,
      range
    );

    for (String entityId : entitiesInRange) {
      Entity entity = entityRegistry.get(entityId);
      if (entity != null && entity.type == type && !entity.isStale()) {
        result.add(entity.position);
      }
    }
    return result;
  }

  public Set<Point> getEntitiesOfType(EntityType type, String subType) {
    return typeIndex
      .get(type)
      .values()
      .stream()
      .filter(
        e -> !e.isStale() && (subType == null || subType.equals(e.details))
      )
      .map(e -> e.position)
      .collect(Collectors.toSet());
  }

  // Maintenance
  public void clearStaleEntities() {
    Set<String> staleEntities = entityRegistry
      .values()
      .stream()
      .filter(Entity::isStale)
      .map(e -> e.id)
      .collect(Collectors.toSet());

    for (String entityId : staleEntities) {
      Entity entity = entityRegistry.get(entityId);
      if (entity != null) {
        entityRegistry.remove(entityId);
        typeIndex.get(entity.type).remove(entityId);
        spatialGrid.remove(entity.position, entityId);
      }
    }
  }

  public boolean isObstacle(Point position) {
    return (
      staticObstacles.containsKey(position) ||
      dynamicObstacles.containsKey(position)
    );
  }

  public void updateDynamicObstacle(
    Point relativePos,
    String type,
    Point currentPos
  ) {
    Point absPos = new Point(
      currentPos.x + relativePos.x,
      currentPos.y + relativePos.y
    );

    if (
      Math.abs(relativePos.x) > AWARENESS_DISTANCE ||
      Math.abs(relativePos.y) > AWARENESS_DISTANCE
    ) {
      return;
    }

    dynamicObstacles.compute(
      absPos,
      (k, existing) -> {
        if (existing != null) {
          existing.updateSeen();
          return existing;
        } else {
          return new ObstacleInfo(absPos, type, true);
        }
      }
    );
  }

  public void clearStaleDynamicObstacles() {
    dynamicObstacles.entrySet().removeIf(entry -> entry.getValue().isStale());
  }

  public boolean isDynamicObstacleInPath(Point from, Point to) {
    clearStaleDynamicObstacles();

    // Check current and predicted positions
    for (ObstacleInfo obstacle : dynamicObstacles.values()) {
      // Check current position
      if (isPointInPath(obstacle.position, from, to)) {
        return true;
      }

      // Check predicted positions (next 2 steps)
      for (int step = 1; step <= 2; step++) {
        Point predicted = obstacle.position;
        if (isPointInPath(predicted, from, to)) {
          return true;
        }
      }
    }
    return false;
  }

  private boolean isPointInPath(Point point, Point from, Point to) {
    // Calculate if point is very close to the path
    double distance = pointToLineDistance(point, from, to);
    return distance < CRITICAL_DISTANCE;
  }

  private double pointToLineDistance(
    Point point,
    Point lineStart,
    Point lineEnd
  ) {
    double normalLength = Math.sqrt(
      (lineEnd.x - lineStart.x) *
      (lineEnd.x - lineStart.x) +
      (lineEnd.y - lineStart.y) *
      (lineEnd.y - lineStart.y)
    );

    if (normalLength == 0.0) return euclideanDistance(point, lineStart);

    double t = Math.max(
      0,
      Math.min(
        1,
        (
          (point.x - lineStart.x) *
          (lineEnd.x - lineStart.x) +
          (point.y - lineStart.y) *
          (lineEnd.y - lineStart.y)
        ) /
        (normalLength * normalLength)
      )
    );

    Point projection = new Point(
      (int) (lineStart.x + t * (lineEnd.x - lineStart.x)),
      (int) (lineStart.y + t * (lineEnd.y - lineStart.y))
    );

    return euclideanDistance(point, projection);
  }

  private double euclideanDistance(Point p1, Point p2) {
    if (p1 == null || p2 == null) return Double.MAX_VALUE;
    double dx = p1.x - p2.x;
    double dy = p1.y - p2.y;
    return Math.sqrt(dx * dx + dy * dy);
  }

  public Map<Point, ObstacleInfo> getDynamicObstacles() {
    // Clean up stale obstacles first
    long now = System.currentTimeMillis();
    dynamicObstacles
      .entrySet()
      .removeIf(entry -> now - entry.getValue().lastSeen > 5000);
    return new HashMap<>(dynamicObstacles);
  }

  public boolean isPathSafe(Point from, Point to) {
    // Check static obstacles
    if (hasObstacleInPath(from, to)) return false;

    // Check dynamic obstacles only if they're close enough to matter
    if (euclideanDistance(from, to) <= AWARENESS_DISTANCE) {
      return !isDynamicObstacleInPath(from, to);
    }

    return true;
  }

  @Override
  public String toString() {
    StringBuilder sb = new StringBuilder();
    sb.append("=== LocalMap State ===\n");

    // Current position
    sb.append(
      String.format(
        "Agent Position: (%d,%d)\n",
        currentPosition.x,
        currentPosition.y
      )
    );
    sb.append("--------------------\n");

    // Dispensers
    sb.append("Dispensers:\n");
    typeIndex
      .get(EntityType.DISPENSER)
      .values()
      .stream()
      .filter(e -> !e.isStale())
      .sorted(
        (e1, e2) -> {
          int xCompare = Integer.compare(e1.position.x, e2.position.x);
          return xCompare != 0
            ? xCompare
            : Integer.compare(e1.position.y, e2.position.y);
        }
      )
      .forEach(
        e -> {
          String debugInfo = DEBUG ? debugTrackingMap.get(e.id).toString() : "";
          sb.append(
            String.format(
              "(%d,%d) %s %s\n",
              e.position.x,
              e.position.y,
              e.details,
              DEBUG ? "[" + debugInfo + "]" : ""
            )
          );
        }
      );

    // Obstacles
    sb.append("\nObstacles:\n");
    obstacles
      .stream()
      .sorted(
        (p1, p2) -> {
          int xCompare = Integer.compare(p1.x, p2.x);
          return xCompare != 0 ? xCompare : Integer.compare(p1.y, p2.y);
        }
      )
      .forEach(p -> sb.append(String.format("(%d,%d)\n", p.x, p.y)));

    // Goals
    sb.append("\nGoals:\n");
    typeIndex
      .get(EntityType.GOAL)
      .values()
      .stream()
      .filter(e -> !e.isStale())
      .sorted(
        (e1, e2) -> {
          int xCompare = Integer.compare(e1.position.x, e2.position.x);
          return xCompare != 0
            ? xCompare
            : Integer.compare(e1.position.y, e2.position.y);
        }
      )
      .forEach(
        e -> {
          String debugInfo = DEBUG ? debugTrackingMap.get(e.id).toString() : "";
          sb.append(
            String.format(
              "(%d,%d)%s\n",
              e.position.x,
              e.position.y,
              DEBUG ? " [" + debugInfo + "]" : ""
            )
          );
        }
      );

    // Statistics
    sb.append("\nStatistics:\n");
    sb.append(
      String.format(
        "Total Dispensers: %d\n",
        typeIndex
          .get(EntityType.DISPENSER)
          .values()
          .stream()
          .filter(e -> !e.isStale())
          .count()
      )
    );
    sb.append(String.format("Total Obstacles: %d\n", obstacles.size()));
    sb.append(
      String.format(
        "Total Goals: %d\n",
        typeIndex
          .get(EntityType.GOAL)
          .values()
          .stream()
          .filter(e -> !e.isStale())
          .count()
      )
    );

    return sb.toString();
  }

  public boolean isOutOfBounds(Point pos) {
    // Check against confirmed boundaries
    for (Map.Entry<String, BoundaryInfo> entry : confirmedBoundaries.entrySet()) {
      String direction = entry.getKey();
      Point boundary = entry.getValue().position;

      switch (direction) {
        case "n":
          if (pos.y <= boundary.y) return true;
          break;
        case "s":
          if (pos.y >= boundary.y) return true;
          break;
        case "w":
          if (pos.x <= boundary.x) return true;
          break;
        case "e":
          if (pos.x >= boundary.x) return true;
          break;
      }
    }
    return false;
  }

  public void updateBoundary(String direction, Point currentPos) {
    // Increment confirmation count for this boundary
    boundaryConfirmations.merge(direction, 1, Integer::sum);

    // Only update bounds after multiple confirmations
    if (boundaryConfirmations.get(direction) >= MIN_BOUNDARY_CONFIRMATIONS) {
      if (!boundsInitialized) {
        mapMinBounds = new Point(currentPos.x - 50, currentPos.y - 50); // Initial guess
        mapMaxBounds = new Point(currentPos.x + 50, currentPos.y + 50);
        boundsInitialized = true;
      }

      // Update specific boundary
      switch (direction) {
        case "n":
          mapMinBounds = new Point(mapMinBounds.x, currentPos.y);
          break;
        case "s":
          mapMaxBounds = new Point(mapMaxBounds.x, currentPos.y);
          break;
        case "w":
          mapMinBounds = new Point(currentPos.x, mapMinBounds.y);
          break;
        case "e":
          mapMaxBounds = new Point(currentPos.x, mapMaxBounds.y);
          break;
      }
    }
  }

  public boolean isBoundaryConfirmed(String direction) {
    return (
      boundaryConfirmations.getOrDefault(direction, 0) >=
      MIN_BOUNDARY_CONFIRMATIONS
    );
  }

  public Point getMapMinBounds() {
    return mapMinBounds;
  }

  public Point getMapMaxBounds() {
    return mapMaxBounds;
  }

  public boolean areBoundsInitialized() {
    return boundsInitialized;
  }

  private void checkForBoundaryPattern(int relX, int relY, Point currentPos) {
    // Look for continuous lines of obstacles that might indicate boundaries
    if (Math.abs(relX) == LocalMap.AWARENESS_DISTANCE) {
      // Potential east/west boundary
      String direction = relX > 0 ? "e" : "w";
      updateBoundary(direction, new Point(currentPos.x + relX, currentPos.y));
    }
    if (Math.abs(relY) == LocalMap.AWARENESS_DISTANCE) {
      // Potential north/south boundary
      String direction = relY > 0 ? "s" : "n";
      updateBoundary(direction, new Point(currentPos.x, currentPos.y + relY));
    }
  }

  public Map<String, Point> getBoundaryDistances(Point currentPos) {
    Map<String, Point> distances = new HashMap<>();

    if (boundsInitialized) {
      if (isBoundaryConfirmed("n")) distances.put(
        "n",
        new Point(0, currentPos.y - mapMinBounds.y)
      );
      if (isBoundaryConfirmed("s")) distances.put(
        "s",
        new Point(0, mapMaxBounds.y - currentPos.y)
      );
      if (isBoundaryConfirmed("w")) distances.put(
        "w",
        new Point(currentPos.x - mapMinBounds.x, 0)
      );
      if (isBoundaryConfirmed("e")) distances.put(
        "e",
        new Point(mapMaxBounds.x - currentPos.x, 0)
      );
    }

    return distances;
  }

  public void recordFailedMove(String direction, Point position) {
    BoundaryEvidence evidence = boundaryEvidence.computeIfAbsent(
      direction,
      k -> new BoundaryEvidence()
    );
    evidence.addFailedMove(position);
  }

  public void checkVisionLimitObstacles(
    Collection<Term> percepts,
    Point currentPos
  ) {
    // Reset continuous obstacle counts
    Map<String, Integer> continuousObstacles = new HashMap<>();

    // Check for continuous obstacles at vision limits
    for (Term percept : percepts) {
      if (!(percept instanceof Atom)) continue;

      Atom atom = (Atom) percept;
      if (!"obstacle".equals(atom.getFunctor())) continue;

      try {
        // Get relative coordinates
        List<Term> terms = atom.getTerms();
        if (terms == null || terms.size() < 2) continue;

        int relX = (int) ((NumberTerm) terms.get(0)).solve();
        int relY = (int) ((NumberTerm) terms.get(1)).solve();

        // Check if obstacle is at vision limit
        if (
          Math.abs(relX) == AWARENESS_DISTANCE ||
          Math.abs(relY) == AWARENESS_DISTANCE
        ) {
          String direction = getDirectionFromRelative(relX, relY);
          continuousObstacles.merge(direction, 1, Integer::sum);
        }
      } catch (Exception e) {
        // Skip invalid percepts
      }
    }

    // Update evidence for each direction
    for (Map.Entry<String, Integer> entry : continuousObstacles.entrySet()) {
      BoundaryEvidence evidence = boundaryEvidence.computeIfAbsent(
        entry.getKey(),
        k -> new BoundaryEvidence()
      );
      evidence.addObstacleEvidence(entry.getValue());
    }
  }

  private String getDirectionFromRelative(int relX, int relY) {
    if (Math.abs(relX) > Math.abs(relY)) {
      return relX > 0 ? "e" : "w";
    } else {
      return relY > 0 ? "s" : "n";
    }
  }

  public Point getLastConfirmedBoundary(String direction) {
    BoundaryEvidence evidence = boundaryEvidence.get(direction);
    return evidence != null ? evidence.lastConfirmedPosition : null;
  }

  public void recordBoundary(String direction, Point currentPos) {
    // For failed_forbidden failures, we can immediately confirm the boundary
    Point boundaryPos;

    // Create new Point object with adjusted coordinates
    switch (direction) {
      case "n":
        boundaryPos = new Point(currentPos.x, currentPos.y - 1);
        break;
      case "s":
        boundaryPos = new Point(currentPos.x, currentPos.y + 1);
        break;
      case "e":
        boundaryPos = new Point(currentPos.x + 1, currentPos.y);
        break;
      case "w":
        boundaryPos = new Point(currentPos.x - 1, currentPos.y);
        break;
      default:
        return;
    }

    confirmedBoundaries.put(direction, new BoundaryInfo(boundaryPos));

    if (DEBUG) {
      System.out.println(
        "Confirmed boundary " + direction + " at " + boundaryPos
      );
    }
  }

  public void recordObstacle(Point position, String type, boolean isDynamic) {
    // Add as an entity using relative coordinates
    Point relativePos = new Point(
      position.x - currentPosition.x,
      position.y - currentPosition.y
    );
    addEntity(EntityType.OBSTACLE, type, relativePos, currentPosition);

    // Maintain legacy collections
    if (isDynamic) {
      dynamicObstacles.put(position, new ObstacleInfo(position, type, true));
    } else {
      staticObstacles.put(position, new ObstacleInfo(position, type, false));
      obstacles.add(position);
    }
  }

  public Point getBoundaryPosition(String direction) {
    BoundaryInfo info = confirmedBoundaries.get(direction);
    return info != null ? info.position : null;
  }

  public Map<String, BoundaryInfo> getConfirmedBoundaries() {
    return new HashMap<>(confirmedBoundaries);
  }

  public Map<String, Point> getConfirmedBoundariesPositions() {
    Map<String, Point> positions = new HashMap<>();
    for (Map.Entry<String, BoundaryInfo> entry : confirmedBoundaries.entrySet()) {
      positions.put(entry.getKey(), entry.getValue().position);
    }
    return positions;
  }

  public void updateFromPercepts(Collection<Term> percepts, Point currentPos) {
    Set<Point> currentlyVisible = new HashSet<>();

    for (Term percept : percepts) {
      if (!(percept instanceof Atom)) continue;

      try {
        String name = ((Atom) percept).getFunctor();
        List<Term> terms = ((Atom) percept).getTerms();

        if (terms == null || terms.size() < 2) continue;

        double x = ((NumberTerm) terms.get(0)).solve();
        double y = ((NumberTerm) terms.get(1)).solve();
        Point absPos = new Point(
          currentPos.x + (int) x,
          currentPos.y + (int) y
        );

        if ("thing".equals(name) && terms.size() > 2) {
          String type = terms.get(2).toString();
          if ("entity".equals(type)) {
            currentlyVisible.add(absPos);
            dynamicObstacles
              .computeIfAbsent(
                absPos,
                k -> new ObstacleInfo(absPos, type, true)
              )
              .updateSeen();
          }
        }
      } catch (Exception e) {
        // Skip invalid percepts
      }
    }

    // Clean up stale obstacles
    long now = System.currentTimeMillis();
    dynamicObstacles
      .entrySet()
      .removeIf(
        entry ->
          now - entry.getValue().lastSeen > 5000 &&
          !currentlyVisible.contains(entry.getKey())
      );
  }

  public boolean isForbidden(Point pos) {
    if (DEBUG) {
      logger.info(
        String.format(
          "Checking if position %s is forbidden. Current boundaries: %s",
          pos,
          getConfirmedBoundariesPositions()
        )
      );
    }

    // First check static obstacles
    if (hasObstacle(pos)) {
      if (DEBUG) {
        logger.info(String.format("Position %s has a static obstacle", pos));
      }
      return true;
    }

    // Then check boundary planes
    Map<String, Point> boundaries = getConfirmedBoundariesPositions();
    if (!boundaries.isEmpty()) {
      for (Map.Entry<String, Point> entry : boundaries.entrySet()) {
        String direction = entry.getKey();
        Point boundaryPoint = entry.getValue();

        boolean isBeyondBoundary = false;

        // Check if position is strictly beyond the boundary plane
        switch (direction) {
          case "n":
            isBeyondBoundary = pos.y < boundaryPoint.y;
            break;
          case "s":
            isBeyondBoundary = pos.y > boundaryPoint.y;
            break;
          case "e":
            isBeyondBoundary = pos.x > boundaryPoint.x;
            break;
          case "w":
            isBeyondBoundary = pos.x < boundaryPoint.x;
            break;
        }

        if (isBeyondBoundary) {
          if (DEBUG) {
            logger.info(
              String.format(
                "Position %s is beyond %s boundary at %s",
                pos,
                direction,
                boundaryPoint
              )
            );
          }
          return true;
        }
      }
    }

    return false;
  }

  // Renamed from isOutOfBounds to better reflect its purpose
  public boolean isForbiddenMove(Point pos) {
    return isForbidden(pos);
  }

  public Set<Point> getObstaclesInRange(Point center, int range) {
    Set<Point> result = new HashSet<>();
    for (Point obstaclePos : staticObstacles.keySet()) {
      if (
        Math.abs(obstaclePos.x - center.x) <= range &&
        Math.abs(obstaclePos.y - center.y) <= range
      ) {
        result.add(obstaclePos);
      }
    }
    return result;
  }

  public boolean isDynamicObstacle(Point pos) {
    return dynamicObstacles.containsKey(pos);
  }

  public boolean isStaticObstacle(Point pos) {
    return staticObstacles.containsKey(pos);
  }

  public void recordStaticObstacle(Point position) {
    staticObstacles.put(position, new ObstacleInfo(position, "static", false));
  }

  public void addOtherAgent(int relX, int relY, Point currentPos) {
    Point agentPos = new Point(currentPos.x + relX, currentPos.y + relY);
    dynamicObstacles.put(agentPos, new ObstacleInfo(agentPos, "dynamic", true));

    if (DEBUG) {
      System.out.println("Added dynamic obstacle (agent) at " + agentPos);
    }
  }

  // Helper method to clear dynamic obstacles at the start of each step
  public void clearDynamicObstacles() {
    dynamicObstacles.clear();
  }

  // Get all static obstacles for path planning
  public Set<Point> getStaticObstacles() {
    return new HashSet<>(staticObstacles.keySet());
  }

  public boolean hasObstacle(Point pos) {
    return (
      staticObstacles.containsKey(pos) || dynamicObstacles.containsKey(pos)
    );
  }

  // Add this helper method if you need just the positions:
  public Set<Point> getDynamicObstaclePositions() {
    return new HashSet<>(getDynamicObstacles().keySet());
  }

  // Add this method to check for obstacles along a path
  private boolean hasObstacleInPath(Point from, Point to) {
    // Simple line-of-sight check using Bresenham's algorithm
    int dx = Math.abs(to.x - from.x);
    int dy = Math.abs(to.y - from.y);
    int x = from.x;
    int y = from.y;
    int n = 1 + dx + dy;
    int x_inc = (to.x > from.x) ? 1 : -1;
    int y_inc = (to.y > from.y) ? 1 : -1;
    int error = dx - dy;
    dx *= 2;
    dy *= 2;

    for (; n > 0; --n) {
      Point check = new Point(x, y);
      if (hasObstacle(check)) {
        return true;
      }
      if (error > 0) {
        x += x_inc;
        error -= dy;
      } else {
        y += y_inc;
        error += dx;
      }
    }
    return false;
  }

  public List<Point> getDispensers() {
    return typeIndex
      .get(EntityType.DISPENSER)
      .values()
      .stream()
      .filter(e -> !e.isStale())
      .map(e -> e.position)
      .collect(Collectors.toList());
  }

  public List<Point> getBlocks() {
    return typeIndex
      .get(EntityType.BLOCK)
      .values()
      .stream()
      .filter(e -> !e.isStale())
      .map(e -> e.position)
      .collect(Collectors.toList());
  }

  public List<Point> getGoals() {
    return typeIndex
      .get(EntityType.GOAL)
      .values()
      .stream()
      .filter(e -> !e.isStale())
      .map(e -> e.position)
      .collect(Collectors.toList());
  }

  public void removeEntity(Point pos) {
    dispensers.remove(pos);
    blocks.remove(pos);
    goals.remove(pos);
  }

  public boolean isNearBoundary(Point pos) {
    Map<String, Point> boundaries = getConfirmedBoundariesPositions();
    if (boundaries.isEmpty()) {
      return false;
    }

    for (Point boundary : boundaries.values()) {
      if (
        Math.abs(pos.x - boundary.x) <= AWARENESS_DISTANCE ||
        Math.abs(pos.y - boundary.y) <= AWARENESS_DISTANCE
      ) {
        return true;
      }
    }
    return false;
  }

  public void handleBoundaryFailure(String direction) {
    Point currentPos = getCurrentPosition();
    if (currentPos == null) {
      logger.warning("Cannot handle boundary failure - no current position");
      return;
    }

    // Record boundary plane one step beyond the failure point
    Point boundaryPoint;
    switch (direction) {
      case "n":
        boundaryPoint = new Point(currentPos.x, currentPos.y - 1);
        break;
      case "s":
        boundaryPoint = new Point(currentPos.x, currentPos.y + 1);
        break;
      case "e":
        boundaryPoint = new Point(currentPos.x + 1, currentPos.y);
        break;
      case "w":
        boundaryPoint = new Point(currentPos.x - 1, currentPos.y);
        break;
      default:
        return;
    }

    // Create BoundaryInfo object instead of just using Point
    BoundaryInfo boundaryInfo = new BoundaryInfo(boundaryPoint);
    confirmedBoundaries.put(direction, boundaryInfo);

    if (DEBUG) {
      logger.info(
        String.format(
          "Recorded boundary plane at %s in direction %s (one step beyond current position %s)",
          boundaryPoint,
          direction,
          currentPos
        )
      );
    }
  }

  public void handlePathFailure() {
    try {
      if (lastAttemptedMove != null) {
        recordStaticObstacle(lastAttemptedMove);
        logger.info("Static obstacle recorded at " + lastAttemptedMove);
      }
    } catch (Exception e) {
      logger.severe("Error in handlePathFailure: " + e.getMessage());
    }
  }

  private String getDirectionFromPositions(Point from, Point to) {
    if (from == null || to == null) return null;

    int dx = to.x - from.x;
    int dy = to.y - from.y;

    if (dx > 0) return "e";
    if (dx < 0) return "w";
    if (dy > 0) return "s";
    if (dy < 0) return "n";

    return null;
  }

  private void cleanupTemporaryObstacles() {
    // Decrease TTL and remove expired obstacles
    temporaryObstacles
      .entrySet()
      .removeIf(
        entry -> {
          entry.setValue(entry.getValue() - 1);
          return entry.getValue() <= 0;
        }
      );
  }

  private void updateMapBounds(Point currentPos, String direction) {
    if (direction == null) return;

    // Update map bounds based on confirmed boundary
    switch (direction) {
      case "n":
        if (
          mapMinBounds == null || currentPos.y < mapMinBounds.y
        ) mapMinBounds =
          new Point(
            mapMinBounds != null ? mapMinBounds.x : currentPos.x,
            currentPos.y
          );
        break;
      case "s":
        if (
          mapMaxBounds == null || currentPos.y > mapMaxBounds.y
        ) mapMaxBounds =
          new Point(
            mapMaxBounds != null ? mapMaxBounds.x : currentPos.x,
            currentPos.y
          );
        break;
      case "w":
        if (
          mapMinBounds == null || currentPos.x < mapMinBounds.x
        ) mapMinBounds =
          new Point(
            currentPos.x,
            mapMinBounds != null ? mapMinBounds.y : currentPos.y
          );
        break;
      case "e":
        if (
          mapMaxBounds == null || currentPos.x > mapMaxBounds.x
        ) mapMaxBounds =
          new Point(
            currentPos.x,
            mapMaxBounds != null ? mapMaxBounds.y : currentPos.y
          );
        break;
    }
  }

  // Track attempted moves
  public void setLastAttemptedMove(Point target) {
    this.lastAttemptedMove = target;
  }

  // Add this method to help with debugging
  public void logBoundaryState() {
    if (DEBUG) {
      Point currentPos = getCurrentPosition();
      logger.info(
        String.format(
          "Current position: %s, Boundaries: %s",
          currentPos,
          confirmedBoundaries
            .entrySet()
            .stream()
            .map(
              e -> String.format("%s at %s", e.getKey(), e.getValue().position)
            )
            .collect(Collectors.joining(", "))
        )
      );
    }
  }
}
