package jason.eis;

import eis.iilang.*;
import jason.asSyntax.*;
import java.util.*;
import java.util.concurrent.ConcurrentHashMap;
import java.util.stream.Collectors;

public class LocalMap {
    // Fast lookup maps with initial capacity for better performance
    private final Map<Point, MapObject> dispensers = new HashMap<>(32);
    private final Map<Point, MapObject> obstacles = new HashMap<>(64);
    private final Map<Point, MapObject> agents = new HashMap<>(16);
    private final Map<Point, MapObject> blocks = new HashMap<>(32);
    private final Set<Point> goals = new HashSet<>(16);
    
    // BitSet for explored cells - very fast bit operations
    private final BitSet exploredCells = new BitSet(10000); // Pre-sized for efficiency
    private Point currentPosition = new Point(0, 0);
    
    // Cached values for frequent calculations
    private Point lastTarget = null;
    private List<String> lastPath = null;
    private static final Point[] NEIGHBORS = new Point[] {
        new Point(0, -1),  // N
        new Point(0, 1),   // S
        new Point(1, 0),   // E
        new Point(-1, 0)   // W
    };

    public enum ObjectType {
        DISPENSER,
        BLOCK,
        OBSTACLE,
        AGENT
    }

    public static class MapObject {
        final ObjectType type;
        final long lastSeen;
        final String details;

        MapObject(ObjectType type, String details) {
            this.type = type;
            this.lastSeen = System.currentTimeMillis();
            this.details = details;
        }
    }

    // Fast A* implementation with minimal object creation
    private static final class Node implements Comparable<Node> {
        Point point;
        Node parent;
        int g = Integer.MAX_VALUE;
        int h;
        int f = Integer.MAX_VALUE;

        Node(Point point) {
            this.point = point;
        }

        @Override
        public int compareTo(Node other) {
            return Integer.compare(this.f, other.f);
        }
    }

    public void scanPerceptions(Collection<Percept> percepts) {
        for (Percept p : percepts) {
            if ("thing".equals(p.getName())) {
                Parameter[] params = p.getParameters().toArray(new Parameter[0]);
                int relX = ((Numeral) params[0]).getValue().intValue();
                int relY = ((Numeral) params[1]).getValue().intValue();
                String type = ((Identifier) params[2]).getValue();
                String details = params.length > 3 ? ((Identifier) params[3]).getValue() : "";
                
                switch(type) {
                    case "dispenser":
                        addObject(relX, relY, ObjectType.DISPENSER, details);
                        break;
                    case "block":
                        addObject(relX, relY, ObjectType.BLOCK, details);
                        break;
                    case "obstacle":
                        addObject(relX, relY, ObjectType.OBSTACLE, "");
                        break;
                    case "entity":
                        addObject(relX, relY, ObjectType.AGENT, details);
                        break;
                }
            } else if ("goal".equals(p.getName())) {
                Parameter[] params = p.getParameters().toArray(new Parameter[0]);
                int relX = ((Numeral) params[0]).getValue().intValue();
                int relY = ((Numeral) params[1]).getValue().intValue();
                addGoal(relX, relY);
            }
        }
        clearOldDynamicObjects();
    }

    private void clearOldDynamicObjects() {
        long now = System.currentTimeMillis();
        agents.values().removeIf(obj -> now - obj.lastSeen > 1000); // 1 second timeout for agents
    }

    public void addObject(int relativeX, int relativeY, ObjectType type, String details) {
        Point absolutePoint = new Point(
            currentPosition.x + relativeX,
            currentPosition.y + relativeY
        );
        MapObject obj = new MapObject(type, details);
        
        switch (type) {
            case DISPENSER -> dispensers.put(absolutePoint, obj);
            case BLOCK -> blocks.put(absolutePoint, obj);
            case OBSTACLE -> obstacles.put(absolutePoint, obj);
            case AGENT -> agents.put(absolutePoint, obj);
        }
        markExplored(absolutePoint);
    }

    public void addGoal(int relativeX, int relativeY) {
        Point absolutePoint = new Point(
            currentPosition.x + relativeX,
            currentPosition.y + relativeY
        );
        goals.add(absolutePoint);
    }

    public Point findNearestObject(ObjectType type) {
        Map<Point, MapObject> sourceMap = switch (type) {
            case DISPENSER -> dispensers;
            case OBSTACLE -> obstacles;
            case AGENT -> agents;
            default -> null;
        };

        Point nearest = null;
        int minDistance = Integer.MAX_VALUE;

        for (Point p : sourceMap.keySet()) {
            int distance = manhattanDistance(currentPosition, p);
            if (distance < minDistance) {
                minDistance = distance;
                nearest = p;
            }
        }
        return nearest;
    }

    public List<String> findPathTo(Point target) {
        if (target == null) return Collections.emptyList();
        if (target.equals(lastTarget) && lastPath != null) return new ArrayList<>(lastPath);
        
        lastTarget = target;
        lastPath = computePath(target);
        return new ArrayList<>(lastPath);
    }

    private List<String> computePath(Point target) {
        PriorityQueue<Node> openSet = new PriorityQueue<>(32);
        Map<Point, Node> allNodes = new HashMap<>(64);
        BitSet closedSet = new BitSet(10000);

        Node start = new Node(currentPosition);
        start.g = 0;
        start.h = manhattanDistance(currentPosition, target);
        start.f = start.h;

        openSet.add(start);
        allNodes.put(currentPosition, start);

        while (!openSet.isEmpty()) {
            Node current = openSet.poll();
            if (current.point.equals(target)) {
                return reconstructPath(current);
            }

            int idx = pointToIndex(current.point);
            closedSet.set(idx);

            for (Point neighbor : NEIGHBORS) {
                Point neighborPoint = new Point(
                    current.point.x + neighbor.x,
                    current.point.y + neighbor.y
                );
                
                int neighborIdx = pointToIndex(neighborPoint);
                if (closedSet.get(neighborIdx)) continue;
                if (isBlocked(neighborPoint)) continue;

                int tentativeG = current.g + 1;

                Node neighborNode = allNodes.get(neighborPoint);
                if (neighborNode == null) {
                    neighborNode = new Node(neighborPoint);
                    allNodes.put(neighborPoint, neighborNode);
                }

                if (tentativeG < neighborNode.g) {
                    neighborNode.parent = current;
                    neighborNode.g = tentativeG;
                    neighborNode.h = manhattanDistance(neighborPoint, target);
                    neighborNode.f = neighborNode.g + neighborNode.h;
                    openSet.add(neighborNode);
                }
            }
        }

        return Collections.emptyList();
    }

    private static int manhattanDistance(Point a, Point b) {
        return Math.abs(a.x - b.x) + Math.abs(a.y - b.y);
    }

    private boolean isBlocked(Point p) {
        return obstacles.containsKey(p) || agents.containsKey(p);
    }

    private int pointToIndex(Point p) {
        return (p.x + 50) * 100 + (p.y + 50);
    }

    private void markExplored(Point p) {
        exploredCells.set(pointToIndex(p));
    }

    private List<String> reconstructPath(Node end) {
        List<String> path = new ArrayList<>();
        Node current = end;
        Node parent = current.parent;

        while (parent != null) {
            int dx = current.point.x - parent.point.x;
            int dy = current.point.y - parent.point.y;
            
            path.add(0, dx == 0 ? (dy < 0 ? "n" : "s") : (dx < 0 ? "w" : "e"));
            
            current = parent;
            parent = current.parent;
        }

        return path;
    }

    public void updatePosition(String direction) {
        switch (direction) {
            case "n" -> currentPosition = new Point(currentPosition.x, currentPosition.y - 1);
            case "s" -> currentPosition = new Point(currentPosition.x, currentPosition.y + 1);
            case "e" -> currentPosition = new Point(currentPosition.x + 1, currentPosition.y);
            case "w" -> currentPosition = new Point(currentPosition.x - 1, currentPosition.y);
        }
        markExplored(currentPosition);
        lastPath = null; // Invalidate cached path
    }

    public Point getCurrentPosition() {
        return currentPosition;
    }

    public List<Point> findNearestDispenser(String blockType) {
        return dispensers.entrySet().stream()
            .filter(e -> blockType.equals(e.getValue().details))
            .map(Map.Entry::getKey)
            .sorted(Comparator.comparingInt(p -> 
                Math.abs(p.x - currentPosition.x) + Math.abs(p.y - currentPosition.y)))
            .collect(Collectors.toList());
    }

    public List<Point> findNearestGoal() {
        return goals.stream()
            .sorted(Comparator.comparingInt(p -> 
                Math.abs(p.x - currentPosition.x) + Math.abs(p.y - currentPosition.y)))
            .collect(Collectors.toList());
    }

    public boolean isGoal(Point p) {
        return goals.contains(p);
    }
} 