package jason.eis.movements;

import jason.eis.LocalMap;
import jason.eis.Point;
import java.util.*;
import java.util.concurrent.ConcurrentHashMap;

public class Exploration {
  private static final int ZONE_SIZE = 5;
  private final Map<Point, ZoneInfo> zoneMemory = new ConcurrentHashMap<>();
  private final Map<String, Set<Point>> agentVisitedZones = new ConcurrentHashMap<>();

  private static class ZoneInfo {
    int visits = 0;
    long lastVisitTime = 0;
    double explorationScore = 1.0;

    void visit() {
      visits++;
      lastVisitTime = System.currentTimeMillis();
      updateScore();
    }

    private void updateScore() {
      // Decrease score with visits but maintain minimum
      explorationScore = Math.max(0.2, 1.0 - (visits * 0.2));
    }

    void decay() {
      long timeSinceVisit = System.currentTimeMillis() - lastVisitTime;
      if (timeSinceVisit > 30000) { // 30 seconds
        explorationScore = Math.min(1.0, explorationScore + 0.1);
      }
    }
  }

  public double scoreDirection(
    String agName,
    Point currentPos,
    String direction,
    LocalMap map
  ) {
    Point nextPos = calculateNextPosition(currentPos, direction);
    Point zone = getZone(nextPos);

    // Get or create zone info
    ZoneInfo info = zoneMemory.computeIfAbsent(zone, k -> new ZoneInfo());
    info.decay();

    // Track agent's visited zones
    Set<Point> agentZones = agentVisitedZones.computeIfAbsent(
      agName,
      k -> new HashSet<>()
    );

    // Higher score for unexplored zones
    double explorationScore = agentZones.contains(zone)
      ? info.explorationScore
      : 1.0;

    return explorationScore;
  }

  public void recordVisit(String agName, Point position) {
    Point zone = getZone(position);
    ZoneInfo info = zoneMemory.computeIfAbsent(zone, k -> new ZoneInfo());
    info.visit();

    // Record for this agent
    agentVisitedZones.computeIfAbsent(agName, k -> new HashSet<>()).add(zone);
  }

  private Point getZone(Point position) {
    return new Point(
      (int) Math.floor(position.x / (double) ZONE_SIZE),
      (int) Math.floor(position.y / (double) ZONE_SIZE)
    );
  }

  private Point calculateNextPosition(Point current, String direction) {
    if (direction == null) return current;
    switch (direction) {
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
}
