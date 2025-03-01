package jason.eis.movements;

import jason.eis.LocalMap;
import jason.eis.Point;
import java.util.*;
import java.util.concurrent.ConcurrentHashMap;

public class Exploration {
  private static final int ZONE_SIZE = 5;
  private static final double UNEXPLORED_BONUS = 1.5;
  private static final double REVISIT_PENALTY = 0.4;
  private static final int DECAY_TIME = 30000; // 30 seconds
  private static final double ADJACENT_ZONE_WEIGHT = 0.3;

  private final Map<Point, ZoneInfo> zoneMemory = new ConcurrentHashMap<>();
  private final Map<String, Set<Point>> agentVisitedZones = new ConcurrentHashMap<>();

  private static class ZoneInfo {
    int visits = 0;
    long lastVisitTime = 0;
    double explorationScore = 1.0;
    Set<String> visitingAgents = new HashSet<>();

    void visit(String agName) {
      visits++;
      lastVisitTime = System.currentTimeMillis();
      visitingAgents.add(agName);
      updateScore();
    }

    private void updateScore() {
      // Sharper decrease for frequently visited zones
      explorationScore = Math.max(0.1, 1.0 - (visits * 0.25));
      // Additional penalty for multiple agents visiting same zone
      if (visitingAgents.size() > 1) {
        explorationScore *= 0.8;
      }
    }

    void decay() {
      long timeSinceVisit = System.currentTimeMillis() - lastVisitTime;
      if (timeSinceVisit > DECAY_TIME) {
        explorationScore = Math.min(1.0, explorationScore + 0.15);
        // Clear old visiting agents
        if (timeSinceVisit > DECAY_TIME * 2) {
          visitingAgents.clear();
        }
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
    Point nextZone = getZone(nextPos);

    // Get current zone info
    ZoneInfo info = zoneMemory.computeIfAbsent(nextZone, k -> new ZoneInfo());
    info.decay();

    Set<Point> agentZones = agentVisitedZones.computeIfAbsent(
      agName,
      k -> new HashSet<>()
    );

    // Base score calculation
    double score;
    if (!agentZones.contains(nextZone)) {
      // Bonus for completely new zones
      score = UNEXPLORED_BONUS;
    } else {
      // Penalty for revisiting
      score = info.explorationScore * REVISIT_PENALTY;
    }

    // Consider adjacent zones' exploration status
    double adjacentScore = getAdjacentZonesScore(nextZone, agName);
    score += adjacentScore * ADJACENT_ZONE_WEIGHT;

    // Avoid zones with other agents
    if (
      info.visitingAgents.size() > 0 && !info.visitingAgents.contains(agName)
    ) {
      score *= 0.7;
    }

    return score;
  }

  private double getAdjacentZonesScore(Point zone, String agName) {
    double totalScore = 0;
    int count = 0;

    // Check all adjacent zones
    for (int dx = -1; dx <= 1; dx++) {
      for (int dy = -1; dy <= 1; dy++) {
        if (dx == 0 && dy == 0) continue;

        Point adjacentZone = new Point(zone.x + dx, zone.y + dy);
        ZoneInfo info = zoneMemory.get(adjacentZone);

        if (info == null) {
          // Unexplored adjacent zones are good
          totalScore += 1.0;
        } else {
          totalScore += info.explorationScore;
        }
        count++;
      }
    }

    return count > 0 ? totalScore / count : 0;
  }

  public void recordVisit(String agName, Point position) {
    Point zone = getZone(position);
    ZoneInfo info = zoneMemory.computeIfAbsent(zone, k -> new ZoneInfo());
    info.visit(agName);

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
