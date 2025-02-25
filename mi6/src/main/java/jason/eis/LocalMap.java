package jason.eis;

import eis.iilang.*;
import java.awt.Point;
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
  private Point absolutePosition;
  private Point lastRelativePosition;

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
    this.absolutePosition = new Point(0, 0);
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
  }

  /**
   * Updates the absolute position based on relative movement
   */
  public void updatePosition(Point newRelativePosition) {
    // Calculate the difference in position
    int dx = newRelativePosition.x - lastRelativePosition.x;
    int dy = newRelativePosition.y - lastRelativePosition.y;

    // Update absolute position
    absolutePosition.translate(dx, dy);
    lastRelativePosition = newRelativePosition;

    // Clear stale cache entries
    clearStaleCache();

    logger.fine("Updated position to " + absolutePosition);
  }

  /**
   * Converts relative coordinates to absolute coordinates
   */
  private Point toAbsolute(int relX, int relY) {
    return new Point(absolutePosition.x + relX, absolutePosition.y + relY);
  }

  /**
   * Converts absolute coordinates to relative coordinates
   */
  private Point toRelative(Point absolute) {
    return new Point(
      absolute.x - absolutePosition.x,
      absolute.y - absolutePosition.y
    );
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

    Point absPos = toAbsolute(relX, relY);
    String subType = ((Identifier) params[3]).getValue();
    MapEntity entity = new MapEntity(absPos, type, subType);

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
        updateEntitySet(obstacles, entity);
        break;
      case "goal":
        updateEntitySet(goals, entity);
        break;
    }

    entityMap.put(absPos, entity);
  }

  /**
   * Updates an entity set with new observation
   */
  private void updateEntitySet(
    Map<String, Set<MapEntity>> map,
    MapEntity entity
  ) {
    map
      .computeIfAbsent(
        entity.type,
        k -> Collections.newSetFromMap(new ConcurrentHashMap<>())
      )
      .add(entity);
  }

  /**
   * Clears entities that should be in view but weren't observed
   */
  private void clearOldEntitiesInView() {
    Point pos = absolutePosition;
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

        double distance = getDistance(absolutePosition, entity.position);
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
    String cacheKey = absolutePosition + "->" + target;
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
      absolutePosition,
      null,
      0,
      getDistance(absolutePosition, target)
    );
    openSet.add(start);
    nodeMap.put(absolutePosition, start);

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
    NORTH(0, -1),
    SOUTH(0, 1),
    EAST(1, 0),
    WEST(-1, 0);

    final int dx;
    final int dy;

    Direction(int dx, int dy) {
      this.dx = dx;
      this.dy = dy;
    }
  }

  // Getters for map information
  public Point getAbsolutePosition() {
    return new Point(absolutePosition);
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
}
