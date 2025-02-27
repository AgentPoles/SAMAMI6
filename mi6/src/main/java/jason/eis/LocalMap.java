package jason.eis;

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

      // Only add if not already present
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
    return obstacles.contains(position);
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
}
