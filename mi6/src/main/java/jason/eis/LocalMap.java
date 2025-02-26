package jason.eis;

import eis.iilang.*;
import java.util.*;
import java.util.concurrent.ConcurrentHashMap;
import java.util.logging.Logger;
import java.util.stream.Collectors;

public class LocalMap {
  private static final Logger logger = Logger.getLogger(
    LocalMap.class.getName()
  );

  // Constants for map boundaries
  private static final int MAP_SIZE = 100; // Assumed maximum map size
  private static final int VIEW_DISTANCE = 5; // Maximum view distance

  // Position tracking
  private Point currentPosition; // Current absolute position
  private Point lastRelativePosition; // Last relative position from percepts

  // Separate mappings for specific types and combined
  private final Map<String, Set<MapEntity>> dispensersB0;
  private final Map<String, Set<MapEntity>> dispensersB1;
  private final Map<String, Set<MapEntity>> allDispensers;
  private final Map<String, Set<MapEntity>> blocksB0;
  private final Map<String, Set<MapEntity>> blocksB1;
  private final Map<String, Set<MapEntity>> allBlocks;

  private final Set<MapEntity> obstacles;
  private final Set<MapEntity> goals;
  private final Map<Point, MapEntity> entityMap; // For quick position-based lookups

  // Cache for pathfinding
  private final Map<Point, List<String>> pathCache;
  private static final int PATH_CACHE_SIZE = 100;
  private static final long PATH_CACHE_DURATION = 5000; // 5 seconds

  // Add movement history tracking (only used when DEBUG is true)
  private final List<String> movementHistory;
  private static final int MAX_MOVEMENT_HISTORY = 50; // Keep last 50 moves

  private static final int DUPLICATE_TOLERANCE = 1; // Consider entities within 1 unit as potential duplicates
  private static final long PERCEPTION_WINDOW = 500; // Time window in milliseconds for same perception

  public static boolean DEBUG = true; // Default to false

  // Inner class to represent entities with timestamp
  private static class MapEntity {
    final Point position;
    final String type;
    final String subType; // b0 or b1
    long lastSeen;

    MapEntity(Point position, String type, String subType) {
      this.position = position;
      this.type = type;
      this.subType = subType;
      this.lastSeen = System.currentTimeMillis();
    }

    void updateTimestamp() {
      this.lastSeen = System.currentTimeMillis();
    }

    boolean isStale() {
      return System.currentTimeMillis() - lastSeen > 30000; // 30 seconds
    }

    @Override
    public boolean equals(Object o) {
      if (this == o) return true;
      if (!(o instanceof MapEntity)) return false;
      MapEntity other = (MapEntity) o;
      return (
        position.equals(other.position) &&
        type.equals(other.type) &&
        Objects.equals(subType, other.subType)
      );
    }

    @Override
    public int hashCode() {
      return Objects.hash(position, type, subType);
    }
  }

  public LocalMap() {
    // Start at origin
    this.currentPosition = new Point(0, 0);
    this.lastRelativePosition = new Point(0, 0);

    // Initialize type-specific collections
    this.dispensersB0 = new ConcurrentHashMap<>();
    this.dispensersB1 = new ConcurrentHashMap<>();
    this.allDispensers = new ConcurrentHashMap<>();
    this.blocksB0 = new ConcurrentHashMap<>();
    this.blocksB1 = new ConcurrentHashMap<>();
    this.allBlocks = new ConcurrentHashMap<>();

    this.obstacles = Collections.newSetFromMap(new ConcurrentHashMap<>());
    this.goals = Collections.newSetFromMap(new ConcurrentHashMap<>());
    this.entityMap = new ConcurrentHashMap<>();
    this.pathCache = new ConcurrentHashMap<>();
    this.movementHistory = new ArrayList<>();
  }

  public Point getCurrentPosition() {
    return new Point(currentPosition.x, currentPosition.y);
  }

  public void updatePosition(Point newPosition) {
    if (DEBUG) {
      logger.info(
        String.format(
          "Updating position - Current: (%d,%d), LastRel: (%d,%d), NewRel: (%d,%d)",
          currentPosition.x,
          currentPosition.y,
          lastRelativePosition.x,
          lastRelativePosition.y,
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

    if (DEBUG) {
      logger.info(
        String.format(
          "Position updated - New absolute: (%d,%d), Movement: (%d,%d)",
          currentPosition.x,
          currentPosition.y,
          dx,
          dy
        )
      );
    }
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
      // Add to movement history
      String moveEntry = String.format(
        "Move %s: (%d,%d)",
        direction,
        currentPosition.x,
        currentPosition.y
      );
      movementHistory.add(moveEntry);

      // Keep history size manageable
      if (movementHistory.size() > MAX_MOVEMENT_HISTORY) {
        movementHistory.remove(0);
      }

      logger.info(
        "Updated position to " + currentPosition + " after moving " + direction
      );
    }
  }

  /**
   * Converts a relative position from percepts to absolute position
   * Takes into account our current position relative to origin
   */
  private Point toAbsolutePosition(Point relativePos) {
    Point absolutePos = new Point(
      currentPosition.x + relativePos.x,
      currentPosition.y + relativePos.y
    );

    if (DEBUG) {
      logger.info(
        String.format(
          "Converting position - Relative: (%d,%d), Current: (%d,%d), Absolute: (%d,%d)",
          relativePos.x,
          relativePos.y,
          currentPosition.x,
          currentPosition.y,
          absolutePos.x,
          absolutePos.y
        )
      );
    }

    return absolutePos;
  }

  /**
   * Processes and adds new observations to the map
   */
  public void addObservation(Collection<Percept> percepts) {
    // Clear old entities that are in view range but not seen
    clearOldEntitiesInView();

    for (Percept p : percepts) {
      try {
        processPercept(p);
      } catch (Exception e) {
        logger.warning("Error processing percept: " + e.getMessage());
      }
    }
  }

  /**
   * Processes individual percepts
   */
  private void processPercept(Percept percept) {
    if (!"thing".equals(percept.getName())) return;

    Parameter[] params = percept.getParameters().toArray(new Parameter[0]);
    int relX = ((Numeral) params[0]).getValue().intValue();
    int relY = ((Numeral) params[1]).getValue().intValue();
    String type = ((Identifier) params[2]).getValue();
    String subType = ((Identifier) params[3]).getValue();

    Point relativePos = new Point(relX, relY);
    Point absolutePos = toAbsolutePosition(relativePos);

    if (DEBUG) {
      logger.info(
        String.format(
          "Processing percept - Type: %s, SubType: %s, RelPos: (%d,%d), AbsPos: (%d,%d)",
          type,
          subType,
          relativePos.x,
          relativePos.y,
          absolutePos.x,
          absolutePos.y
        )
      );
    }

    MapEntity entity = new MapEntity(absolutePos, type, subType);

    switch (type) {
      case "dispenser":
        if ("b0".equals(subType)) {
          updateEntitySet(dispensersB0, entity);
        } else if ("b1".equals(subType)) {
          updateEntitySet(dispensersB1, entity);
        }
        updateEntitySet(allDispensers, entity);
        break;
      case "block":
        if ("b0".equals(subType)) {
          updateEntitySet(blocksB0, entity);
        } else if ("b1".equals(subType)) {
          updateEntitySet(blocksB1, entity);
        }
        updateEntitySet(allBlocks, entity);
        break;
      case "obstacle":
        updateSimpleEntitySet(obstacles, entity);
        break;
      case "goal":
        updateSimpleEntitySet(goals, entity);
        break;
    }

    entityMap.put(absolutePos, entity);
  }

  /**
   * Updates an entity set with new observation, considering both position and timing
   */
  private void updateEntitySet(
    Map<String, Set<MapEntity>> map,
    MapEntity newEntity
  ) {
    Set<MapEntity> entities = map.computeIfAbsent(
      newEntity.type,
      k -> Collections.newSetFromMap(new ConcurrentHashMap<>())
    );

    // First, check for exact position matches
    Optional<MapEntity> exactMatch = entities
      .stream()
      .filter(
        existing ->
          existing.position.x == newEntity.position.x &&
          existing.position.y == newEntity.position.y &&
          existing.subType.equals(newEntity.subType)
      )
      .findFirst();

    if (exactMatch.isPresent()) {
      // Update timestamp of exact match
      exactMatch.get().updateTimestamp();
      if (DEBUG) {
        logger.info(
          String.format(
            "Updated timestamp for existing %s at (%d,%d)",
            newEntity.type,
            newEntity.position.x,
            newEntity.position.y
          )
        );
      }
      return;
    }

    // Check for nearby entities that might be duplicates
    Set<MapEntity> nearbyEntities = entities
      .stream()
      .filter(
        existing -> {
          int dx = Math.abs(existing.position.x - newEntity.position.x);
          int dy = Math.abs(existing.position.y - newEntity.position.y);
          long timeDiff = Math.abs(existing.lastSeen - newEntity.lastSeen);

          // Consider it a duplicate if:
          // 1. It's within tolerance
          // 2. It's the same type
          // 3. It was seen very recently
          return (
            dx <= DUPLICATE_TOLERANCE &&
            dy <= DUPLICATE_TOLERANCE &&
            existing.subType.equals(newEntity.subType) &&
            timeDiff < PERCEPTION_WINDOW
          );
        }
      )
      .collect(Collectors.toSet());

    if (nearbyEntities.isEmpty()) {
      // No duplicates found, add as new entity
      if (DEBUG) {
        logger.info(
          String.format(
            "Adding new %s at (%d,%d) [Current pos: (%d,%d)]",
            newEntity.type,
            newEntity.position.x,
            newEntity.position.y,
            currentPosition.x,
            currentPosition.y
          )
        );
      }
      entities.add(newEntity);
    } else {
      // Found potential duplicate(s)
      // Log all nearby entities for debugging
      if (DEBUG) {
        logger.info(
          String.format(
            "Found %d nearby %s entities near (%d,%d):",
            nearbyEntities.size(),
            newEntity.type,
            newEntity.position.x,
            newEntity.position.y
          )
        );
        for (MapEntity nearby : nearbyEntities) {
          logger.info(
            String.format(
              "  - At (%d,%d), seen %d ms ago",
              nearby.position.x,
              nearby.position.y,
              System.currentTimeMillis() - nearby.lastSeen
            )
          );
        }
      }

      // If multiple nearby entities exist, they're probably distinct
      if (nearbyEntities.size() > 1) {
        if (DEBUG) {
          logger.info("Multiple nearby entities found, treating as new entity");
        }
        entities.add(newEntity);
      } else {
        // Single nearby entity, update its position
        MapEntity existing = nearbyEntities.iterator().next();
        if (DEBUG) {
          logger.info(
            String.format(
              "Updating position of existing %s from (%d,%d) to (%d,%d)",
              existing.type,
              existing.position.x,
              existing.position.y,
              newEntity.position.x,
              newEntity.position.y
            )
          );
        }
        entities.remove(existing);
        entities.add(newEntity);
      }
    }
  }

  /**
   * Clears entities that should be in view but weren't observed
   */
  private void clearOldEntitiesInView() {
    Point pos = currentPosition;
    for (int x = -VIEW_DISTANCE; x <= VIEW_DISTANCE; x++) {
      for (int y = -VIEW_DISTANCE; y <= VIEW_DISTANCE; y++) {
        Point checkPos = new Point(pos.x + x, pos.y + y);
        MapEntity entity = entityMap.get(checkPos);
        if (entity != null && entity.isStale()) {
          removeEntity(entity);
        }
      }
    }
  }

  /**
   * Removes an entity from all collections
   */
  private void removeEntity(MapEntity entity) {
    entityMap.remove(entity.position);
    obstacles.remove(entity);
    goals.remove(entity);

    for (Set<MapEntity> dispenserSet : dispensersB0.values()) {
      dispenserSet.remove(entity);
    }
    for (Set<MapEntity> dispenserSet : dispensersB1.values()) {
      dispenserSet.remove(entity);
    }
    for (Set<MapEntity> blockSet : blocksB0.values()) {
      blockSet.remove(entity);
    }
    for (Set<MapEntity> blockSet : blocksB1.values()) {
      blockSet.remove(entity);
    }
  }

  /**
   * Finds the nearest dispenser of a specific type
   */
  public List<Point> findNearestDispenser(String type) {
    Map<String, Set<MapEntity>> sourceMap;
    if (type == null) {
      sourceMap = allDispensers;
    } else if ("b0".equals(type)) {
      sourceMap = dispensersB0;
    } else if ("b1".equals(type)) {
      sourceMap = dispensersB1;
    } else {
      return Collections.emptyList();
    }

    return findNearest(sourceMap);
  }

  /**
   * Finds the nearest block of a specific type
   */
  public List<Point> findNearestBlock(String type) {
    Map<String, Set<MapEntity>> sourceMap;
    if (type == null) {
      sourceMap = allBlocks;
    } else if ("b0".equals(type)) {
      sourceMap = blocksB0;
    } else if ("b1".equals(type)) {
      sourceMap = blocksB1;
    } else {
      return Collections.emptyList();
    }

    return findNearest(sourceMap);
  }

  private List<Point> findNearest(Map<String, Set<MapEntity>> entityMap) {
    List<Point> nearest = new ArrayList<>();
    double minDistance = Double.MAX_VALUE;

    for (Set<MapEntity> entities : entityMap.values()) {
      for (MapEntity entity : entities) {
        if (entity.isStale()) continue;

        double distance = getDistance(currentPosition, entity.position);
        if (distance < minDistance) {
          minDistance = distance;
          nearest.clear();
          nearest.add(entity.position);
        } else if (distance == minDistance) {
          nearest.add(entity.position);
        }
      }
    }

    return nearest;
  }

  /**
   * Calculates Manhattan distance between two points
   */
  private double getDistance(Point a, Point b) {
    return Math.sqrt(Math.pow(b.x - a.x, 2) + Math.pow(b.y - a.y, 2));
  }

  /**
   * Finds a path to the target position
   */
  public List<String> findPathTo(Point target) {
    // Check cache first
    String cacheKey = currentPosition + "->" + target;
    List<String> cachedPath = pathCache.get(target);
    if (cachedPath != null) {
      return new ArrayList<>(cachedPath);
    }

    // Implement A* pathfinding
    List<String> path = findPathAStar(target);

    // Cache the result
    if (pathCache.size() >= PATH_CACHE_SIZE) {
      // Remove oldest entry
      pathCache.remove(pathCache.keySet().iterator().next());
    }
    pathCache.put(target, new ArrayList<>(path));

    return path;
  }

  /**
   * Implements A* pathfinding algorithm
   */
  private List<String> findPathAStar(Point target) {
    PriorityQueue<Node> openSet = new PriorityQueue<>();
    Set<Point> closedSet = new HashSet<>();
    Map<Point, Node> nodeMap = new HashMap<>();

    Node start = new Node(
      currentPosition,
      null,
      0,
      getDistance(currentPosition, target)
    );
    openSet.add(start);
    nodeMap.put(currentPosition, start);

    while (!openSet.isEmpty()) {
      Node current = openSet.poll();

      if (current.position.equals(target)) {
        return reconstructPath(current);
      }

      closedSet.add(current.position);

      // Check all four directions
      for (Direction dir : Direction.values()) {
        Point nextPos = new Point(
          current.position.x + dir.dx,
          current.position.y + dir.dy
        );

        if (closedSet.contains(nextPos) || isObstacle(nextPos)) {
          continue;
        }

        double newG = current.g + 1;
        Node neighbor = nodeMap.get(nextPos);

        if (neighbor == null) {
          neighbor =
            new Node(nextPos, current, newG, getDistance(nextPos, target));
          nodeMap.put(nextPos, neighbor);
          openSet.add(neighbor);
        } else if (newG < neighbor.g) {
          neighbor.parent = current;
          neighbor.g = newG;
          neighbor.f = newG + neighbor.h;
          openSet.remove(neighbor);
          openSet.add(neighbor);
        }
      }
    }

    return Collections.emptyList(); // No path found
  }

  private boolean isObstacle(Point p) {
    MapEntity entity = entityMap.get(p);
    return entity != null && "obstacle".equals(entity.type);
  }

  private List<String> reconstructPath(Node end) {
    List<String> path = new ArrayList<>();
    Node current = end;
    Node parent = current.parent;

    while (parent != null) {
      int dx = current.position.x - parent.position.x;
      int dy = current.position.y - parent.position.y;

      if (dx == 1) path.add(0, "e"); else if (dx == -1) path.add(
        0,
        "w"
      ); else if (dy == 1) path.add(0, "s"); else if (dy == -1) path.add(
        0,
        "n"
      );

      current = parent;
      parent = current.parent;
    }

    return path;
  }

  private void clearStaleCache() {
    long now = System.currentTimeMillis();
    pathCache
      .entrySet()
      .removeIf(
        entry -> now - entry.getValue().get(0).hashCode() > PATH_CACHE_DURATION
      );
  }

  // Helper classes for pathfinding
  private static class Node implements Comparable<Node> {
    Point position;
    Node parent;
    double g; // Cost from start to current
    double h; // Estimated cost to goal
    double f; // g + h

    Node(Point position, Node parent, double g, double h) {
      this.position = position;
      this.parent = parent;
      this.g = g;
      this.h = h;
      this.f = g + h;
    }

    @Override
    public int compareTo(Node other) {
      return Double.compare(this.f, other.f);
    }
  }

  private enum Direction {
    NORTH(0, -1), // Correct: north decrements y
    SOUTH(0, 1), // Correct: south increments y
    EAST(1, 0), // Correct: east increments x
    WEST(-1, 0); // Correct: west decrements x

    final int dx;
    final int dy;

    Direction(int dx, int dy) {
      this.dx = dx;
      this.dy = dy;
    }
  }

  // Getters for map information
  public Point getAbsolutePosition() {
    return new Point(currentPosition.x, currentPosition.y);
  }

  public Set<Point> getObstacles() {
    return obstacles
      .stream()
      .map(entity -> entity.position)
      .collect(Collectors.toSet());
  }

  public Set<Point> getGoals() {
    return goals
      .stream()
      .map(entity -> entity.position)
      .collect(Collectors.toSet());
  }

  public Map<String, Set<Point>> getDispensers() {
    Map<String, Set<Point>> result = new HashMap<>();
    dispensersB0.forEach(
      (type, entities) ->
        result.put(
          type,
          entities
            .stream()
            .map(entity -> entity.position)
            .collect(Collectors.toSet())
        )
    );
    dispensersB1.forEach(
      (type, entities) ->
        result.put(
          type,
          entities
            .stream()
            .map(entity -> entity.position)
            .collect(Collectors.toSet())
        )
    );
    return result;
  }

  public Map<String, Set<Point>> getBlocks() {
    Map<String, Set<Point>> result = new HashMap<>();
    blocksB0.forEach(
      (type, entities) ->
        result.put(
          type,
          entities
            .stream()
            .map(entity -> entity.position)
            .collect(Collectors.toSet())
        )
    );
    blocksB1.forEach(
      (type, entities) ->
        result.put(
          type,
          entities
            .stream()
            .map(entity -> entity.position)
            .collect(Collectors.toSet())
        )
    );
    return result;
  }

  @Override
  public String toString() {
    StringBuilder sb = new StringBuilder();
    sb.append("LocalMap:\n");

    // Dispensers with relative coordinates
    sb.append("Dispensers (Absolute -> Relative):\n");
    for (Map.Entry<String, Set<MapEntity>> entry : dispensersB0.entrySet()) {
      for (MapEntity entity : entry.getValue()) {
        Point relativePos = toRelativePosition(entity.position);
        sb.append(
          String.format(
            "  Location: abs(%d,%d) -> rel(%d,%d), Type: b0\n",
            entity.position.x,
            entity.position.y,
            relativePos.x,
            relativePos.y
          )
        );
      }
    }
    for (Map.Entry<String, Set<MapEntity>> entry : dispensersB1.entrySet()) {
      for (MapEntity entity : entry.getValue()) {
        Point relativePos = toRelativePosition(entity.position);
        sb.append(
          String.format(
            "  Location: abs(%d,%d) -> rel(%d,%d), Type: b1\n",
            entity.position.x,
            entity.position.y,
            relativePos.x,
            relativePos.y
          )
        );
      }
    }

    // Blocks
    sb.append("Blocks:\n");
    for (Map.Entry<String, Set<MapEntity>> entry : blocksB0.entrySet()) {
      for (MapEntity entity : entry.getValue()) {
        sb.append(
          String.format(
            "  Location: (%d,%d), Type: b0\n",
            entity.position.x,
            entity.position.y
          )
        );
      }
    }
    for (Map.Entry<String, Set<MapEntity>> entry : blocksB1.entrySet()) {
      for (MapEntity entity : entry.getValue()) {
        sb.append(
          String.format(
            "  Location: (%d,%d), Type: b1\n",
            entity.position.x,
            entity.position.y
          )
        );
      }
    }

    // Obstacles
    sb.append("Obstacles:\n");
    for (MapEntity entity : obstacles) {
      sb.append(
        String.format(
          "  Location: (%d,%d)\n",
          entity.position.x,
          entity.position.y
        )
      );
    }

    // Current Position
    if (currentPosition != null) {
      sb.append(
        String.format(
          "Current Position: (%d,%d)\n",
          currentPosition.x,
          currentPosition.y
        )
      );
    }

    // Movement history
    if (DEBUG && !movementHistory.isEmpty()) {
      sb
        .append("\nMovement History (last ")
        .append(MAX_MOVEMENT_HISTORY)
        .append(" moves):\n");
      for (String move : movementHistory) {
        sb.append("  ").append(move).append("\n");
      }
    }

    return sb.toString();
  }

  /**
   * Convert absolute position to relative position from current position
   */
  private Point toRelativePosition(Point absolutePos) {
    // Calculate relative coordinates from current position
    Point relativePos = new Point(
      absolutePos.x - currentPosition.x, // X: difference in x
      absolutePos.y - currentPosition.y // Y: difference in y (south is positive)
    );

    if (DEBUG) {
      logger.info(
        String.format(
          "Converting absolute (%d,%d) to relative (%d,%d) from current pos (%d,%d)",
          absolutePos.x,
          absolutePos.y,
          relativePos.x,
          relativePos.y,
          currentPosition.x,
          currentPosition.y
        )
      );
    }

    return relativePos;
  }

  public void addDispenser(Point relativePos, String type) {
    Point absolutePos = toAbsolutePosition(relativePos);
    MapEntity entity = new MapEntity(absolutePos, "dispenser", type);

    if (type.equals("b0")) {
      updateEntitySet(dispensersB0, entity);
    } else if (type.equals("b1")) {
      updateEntitySet(dispensersB1, entity);
    }
    updateEntitySet(allDispensers, entity);

    // Only update entityMap if it's a new entity
    if (!entityMap.containsKey(absolutePos)) {
      entityMap.put(absolutePos, entity);
    }
  }

  public void addBlock(Point relativePos, String type) {
    Point absolutePos = toAbsolutePosition(relativePos);
    MapEntity entity = new MapEntity(absolutePos, "block", type);

    if (type.equals("b0")) {
      updateEntitySet(blocksB0, entity);
    } else if (type.equals("b1")) {
      updateEntitySet(blocksB1, entity);
    }
    updateEntitySet(allBlocks, entity);

    // Only update entityMap if it's a new entity
    if (!entityMap.containsKey(absolutePos)) {
      entityMap.put(absolutePos, entity);
    }
  }

  /**
   * Updates a simple entity set with new observation, considering both position and timing
   */
  private void updateSimpleEntitySet(
    Set<MapEntity> entities,
    MapEntity newEntity
  ) {
    // First, check for exact position matches
    Optional<MapEntity> exactMatch = entities
      .stream()
      .filter(
        existing ->
          existing.position.x == newEntity.position.x &&
          existing.position.y == newEntity.position.y
      )
      .findFirst();

    if (exactMatch.isPresent()) {
      // Update timestamp of exact match
      exactMatch.get().updateTimestamp();
      if (DEBUG) {
        logger.info(
          String.format(
            "Updated timestamp for existing %s at (%d,%d)",
            newEntity.type,
            newEntity.position.x,
            newEntity.position.y
          )
        );
      }
      return;
    }

    // Check for nearby entities that might be duplicates
    Set<MapEntity> nearbyEntities = entities
      .stream()
      .filter(
        existing -> {
          int dx = Math.abs(existing.position.x - newEntity.position.x);
          int dy = Math.abs(existing.position.y - newEntity.position.y);
          long timeDiff = Math.abs(existing.lastSeen - newEntity.lastSeen);

          return (
            dx <= DUPLICATE_TOLERANCE &&
            dy <= DUPLICATE_TOLERANCE &&
            timeDiff < PERCEPTION_WINDOW
          );
        }
      )
      .collect(Collectors.toSet());

    if (nearbyEntities.isEmpty()) {
      if (DEBUG) {
        logger.info(
          String.format(
            "Adding new %s at (%d,%d) [Current pos: (%d,%d)]",
            newEntity.type,
            newEntity.position.x,
            newEntity.position.y,
            currentPosition.x,
            currentPosition.y
          )
        );
      }
      entities.add(newEntity);
    } else if (nearbyEntities.size() == 1) {
      // Single nearby entity, update its position
      MapEntity existing = nearbyEntities.iterator().next();
      if (DEBUG) {
        logger.info(
          String.format(
            "Updating position of existing %s from (%d,%d) to (%d,%d)",
            existing.type,
            existing.position.x,
            existing.position.y,
            newEntity.position.x,
            newEntity.position.y
          )
        );
      }
      entities.remove(existing);
      entities.add(newEntity);
    } else {
      // Multiple nearby entities, treat as new
      if (DEBUG) {
        logger.info("Multiple nearby entities found, treating as new entity");
      }
      entities.add(newEntity);
    }
  }

  public void addObstacle(Point relativePos) {
    Point absolutePos = toAbsolutePosition(relativePos);
    MapEntity entity = new MapEntity(absolutePos, "obstacle", null);

    updateSimpleEntitySet(obstacles, entity);

    if (!entityMap.containsKey(absolutePos)) {
      entityMap.put(absolutePos, entity);
    }
  }

  public void addGoal(Point relativePos) {
    Point absolutePos = toAbsolutePosition(relativePos);
    MapEntity entity = new MapEntity(absolutePos, "goal", null);

    updateSimpleEntitySet(goals, entity);

    if (!entityMap.containsKey(absolutePos)) {
      entityMap.put(absolutePos, entity);
    }
  }
}
