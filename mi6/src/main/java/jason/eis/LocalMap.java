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
  private Point lastRelativePosition;

  // Efficient spatial indexing
  private final SpatialGrid spatialGrid;

  // Type-based indexing for quick entity type access
  private final Map<EntityType, Map<String, Entity>> typeIndex;

  // Entity registry for quick entity lookup by ID
  private final Map<String, Entity> entityRegistry;

  private final Set<Point> obstacles;

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
    long lastSeen;

    Entity(String id, EntityType type, String subType, Point position) {
      this.id = id;
      this.type = type;
      this.subType = subType;
      this.position = position;
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
  }

  private static class SpatialGrid {
    private final Map<Long, Set<String>> grid = new ConcurrentHashMap<>();

    private long hashPos(Point p) {
      // Combine x,y into single long for efficient grid lookup
      // Shift by CELL_SIZE (8) for efficient cell division
      return ((long) (p.x >> 3) << 32) | ((p.y >> 3) & 0xFFFFFFFFL);
    }

    void add(Point p, String entityId) {
      grid
        .computeIfAbsent(hashPos(p), k -> ConcurrentHashMap.newKeySet())
        .add(entityId);
    }

    void remove(Point p, String entityId) {
      Set<String> cell = grid.get(hashPos(p));
      if (cell != null) {
        cell.remove(entityId);
        if (cell.isEmpty()) {
          grid.remove(hashPos(p));
        }
      }
    }

    void move(Point oldPos, Point newPos, String entityId) {
      remove(oldPos, entityId);
      add(newPos, entityId);
    }

    Set<String> getEntitiesInRange(Point center, int range) {
      Set<String> result = ConcurrentHashMap.newKeySet();
      int gridRange = (range + CELL_SIZE - 1) / CELL_SIZE;

      for (int x = -gridRange; x <= gridRange; x++) {
        for (int y = -gridRange; y <= gridRange; y++) {
          Point p = new Point(center.x + (x << 3), center.y + (y << 3));
          Set<String> cell = grid.get(hashPos(p));
          if (cell != null) {
            result.addAll(cell);
          }
        }
      }
      return result;
    }
  }

  public LocalMap() {
    this.obstacles = new HashSet<>();
    this.currentPosition = new Point(0, 0);
    this.lastRelativePosition = new Point(0, 0);
    this.spatialGrid = new SpatialGrid();
    this.typeIndex = new EnumMap<>(EntityType.class);
    this.entityRegistry = new ConcurrentHashMap<>();

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

    // Calculate movement from last relative position
    int dx = newPosition.x - lastRelativePosition.x;
    int dy = newPosition.y - lastRelativePosition.y;

    // Update absolute position
    currentPosition = new Point(currentPosition.x + dx, currentPosition.y + dy);
    lastRelativePosition = newPosition;
  }

  public void updatePositionFromMovement(String direction) {
    switch (direction.toLowerCase()) {
      case "n":
        currentPosition = new Point(currentPosition.x, currentPosition.y - 1);
        break;
      case "s":
        currentPosition = new Point(currentPosition.x, currentPosition.y + 1);
        break;
      case "e":
        currentPosition = new Point(currentPosition.x + 1, currentPosition.y);
        break;
      case "w":
        currentPosition = new Point(currentPosition.x - 1, currentPosition.y);
        break;
    }

    if (DEBUG) {
      logger.fine(
        String.format(
          "Updated position to (%d,%d) after moving %s",
          currentPosition.x,
          currentPosition.y,
          direction
        )
      );
    }
  }

  // Entity management
  private String generateEntityId(EntityType type, Point pos) {
    return String.format("%s_%d_%d_%d", type, pos.x, pos.y, System.nanoTime());
  }

  private void addEntity(EntityType type, String subType, Point relativePos) {
    Point absolutePos = toAbsolutePosition(relativePos);
    String entityId = generateEntityId(type, absolutePos);

    // Create new entity
    Entity entity = new Entity(entityId, type, subType, absolutePos);

    // Update all indexes atomically
    entityRegistry.put(entityId, entity);
    typeIndex.get(type).put(entityId, entity);
    spatialGrid.add(absolutePos, entityId);

    if (DEBUG) {
      logger.fine(
        String.format(
          "Added %s entity at (%d,%d)",
          type,
          absolutePos.x,
          absolutePos.y
        )
      );
    }
  }

  // Public API for adding entities
  public void addDispenser(Point relativePos, String subType) {
    addEntity(EntityType.DISPENSER, subType, relativePos);
  }

  public void addBlock(Point relativePos, String subType) {
    addEntity(EntityType.BLOCK, subType, relativePos);
  }

  public void addObstacle(Point position) {
    obstacles.add(position);
  }

  public void addGoal(Point relativePos) {
    addEntity(EntityType.GOAL, null, relativePos);
  }

  // Coordinate conversion
  private Point toAbsolutePosition(Point relativePos) {
    return new Point(
      currentPosition.x + relativePos.x,
      currentPosition.y + relativePos.y
    );
  }

  private Point toRelativePosition(Point absolutePos) {
    return new Point(
      absolutePos.x - currentPosition.x,
      absolutePos.y - currentPosition.y
    );
  }

  // Query methods for external search algorithms
  public Set<Point> getEntitiesInRange(
    Point center,
    int range,
    EntityType type
  ) {
    Set<Point> result = new HashSet<>();
    Set<String> entitiesInRange = spatialGrid.getEntitiesInRange(center, range);

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
    sb
      .append("LocalMap State for position: ")
      .append(currentPosition)
      .append("\n");

    // List all dispensers with their types
    Map<String, Entity> dispensers = typeIndex.get(EntityType.DISPENSER);
    sb.append("Dispensers (").append(dispensers.size()).append("):\n");
    dispensers
      .values()
      .stream()
      .filter(e -> !e.isStale())
      .forEach(
        e ->
          sb
            .append("  ")
            .append(e.subType)
            .append(" at ")
            .append(e.position)
            .append("\n")
      );

    // List all blocks with their types
    Map<String, Entity> blocks = typeIndex.get(EntityType.BLOCK);
    sb.append("\nBlocks (").append(blocks.size()).append("):\n");
    blocks
      .values()
      .stream()
      .filter(e -> !e.isStale())
      .forEach(
        e ->
          sb
            .append("  ")
            .append(e.subType)
            .append(" at ")
            .append(e.position)
            .append("\n")
      );

    // List all obstacles
    sb.append("\nObstacles (").append(obstacles.size()).append("):\n");
    obstacles.forEach(pos -> sb.append("  ").append(pos).append("\n"));

    // List all goals
    Map<String, Entity> goals = typeIndex.get(EntityType.GOAL);
    sb.append("\nGoals (").append(goals.size()).append("):\n");
    goals
      .values()
      .stream()
      .filter(e -> !e.isStale())
      .forEach(e -> sb.append("  ").append(e.position).append("\n"));

    return sb.toString();
  }
}
