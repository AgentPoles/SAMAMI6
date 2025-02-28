package jason.eis;

import jason.asSyntax.Atom;
import jason.asSyntax.NumberTerm;
import jason.asSyntax.Term;
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
  public static boolean DEBUG = true;

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

  public enum EntityType {
    DISPENSER,
    BLOCK,
    OBSTACLE,
    GOAL,
  }

  private static class Entity {
    final String id;
    final EntityType type;
    final String subType; // b0/b1 for blocks/dispensers
    Point position;
    Point agentPosWhenAdded;
    long lastSeen;

    Entity(
      String id,
      EntityType type,
      String subType,
      Point position,
      Point agentPos
    ) {
      this.id = id;
      this.type = type;
      this.subType = subType;
      this.position = position;
      this.agentPosWhenAdded = agentPos;
      this.lastSeen = System.nanoTime();
    }

    void updatePosition(Point newPos) {
      this.position = newPos;
      this.lastSeen = System.nanoTime();
    }

    boolean isStale() {
      return (System.nanoTime() - lastSeen) / 1_000_000 > STALE_THRESHOLD;
    }

    @Override
    public boolean equals(Object o) {
      if (this == o) return true;
      if (!(o instanceof Entity)) return false;
      Entity other = (Entity) o;
      return id.equals(other.id);
    }

    @Override
    public int hashCode() {
      return id.hashCode();
    }

    void updateLastSeen(Point currentPosition) {
      this.lastSeen = System.nanoTime();
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
        existingEntity.updateLastSeen(currentAbsPos);
        return;
      }

      Entity entity = new Entity(
        entityId,
        type,
        subType,
        absolutePos,
        currentAbsPos // Store the agent's position when this entity was added
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
    String subType,
    Point currentAbsPos
  ) {
    addEntity(EntityType.DISPENSER, subType, relativePos, currentAbsPos);
  }

  public void addBlock(Point relativePos, String subType, Point currentAbsPos) {
    addEntity(EntityType.BLOCK, subType, relativePos, currentAbsPos);
  }

  public void addObstacle(Point relativePos, Point currentAbsPos) {
    synchronized (positionLock) {
      Point absolutePos = new Point(
        currentAbsPos.x + relativePos.x,
        currentAbsPos.y + relativePos.y
      );

      // Add to the grid (keep existing functionality)
      if (!obstacles.contains(absolutePos)) {
        obstacles.add(absolutePos);
        if (DEBUG) {
          logger.fine(
            String.format(
              "Added obstacle at (%d,%d) [relative: (%d,%d)]",
              absolutePos.x,
              absolutePos.y,
              relativePos.x,
              relativePos.y
            )
          );
        }
      }

      // Also add to static obstacles info
      staticObstacles.putIfAbsent(
        absolutePos,
        new ObstacleInfo(absolutePos, "static", false)
      );
    }
  }

  public void addGoal(Point relativePos, Point currentAbsPos) {
    addEntity(EntityType.GOAL, null, relativePos, currentAbsPos);
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
        e -> !e.isStale() && (subType == null || subType.equals(e.subType))
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
    return Math.sqrt(
      (p2.x - p1.x) * (p2.x - p1.x) + (p2.y - p1.y) * (p2.y - p1.y)
    );
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
              e.subType,
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

  public void recordObstacle(Point position) {
    recordObstacle(position, "static", false);
  }

  public void recordObstacle(Point position, String type, boolean isDynamic) {
    staticObstacles.put(position, new ObstacleInfo(position, type, isDynamic));
  }

  public Point getBoundaryPosition(String direction) {
    BoundaryInfo info = confirmedBoundaries.get(direction);
    return info != null ? info.position : null;
  }

  public Map<String, Point> getConfirmedBoundaries() {
    Map<String, Point> boundaries = new HashMap<>();
    confirmedBoundaries.forEach(
      (dir, info) -> boundaries.put(dir, info.position)
    );
    return boundaries;
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

    // Check against known obstacles
    return staticObstacles.containsKey(pos);
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
}
