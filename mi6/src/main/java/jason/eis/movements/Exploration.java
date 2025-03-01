package jason.eis.movements;

import jason.eis.LocalMap;
import jason.eis.Point;
import java.util.*;
import java.util.concurrent.ConcurrentHashMap;

public class Exploration {
  private static final int ZONE_SIZE = 5;
  private static final double UNEXPLORED_BONUS = 2.0;
  private static final double REVISIT_PENALTY = 0.3;
  private static final int DECAY_TIME = 30000; // 30 seconds
  private static final double ADJACENT_ZONE_WEIGHT = 0.3;

  // Heat map constants
  private static final double HEAT_DECAY_RATE = 0.92;
  private static final double INITIAL_HEAT = 1.0;
  private static final double MIN_HEAT = 0.1;
  private static final int HEAT_RADIUS = 3;

  // New exploration constants
  private static final double DISTANCE_BONUS_FACTOR = 0.15;
  private static final int MAX_LOCAL_VISITS = 3;
  private static final double LOCAL_AREA_RADIUS = 8;

  private final Map<Point, ZoneInfo> zoneMemory = new ConcurrentHashMap<>();
  private final Map<String, Set<Point>> agentVisitedZones = new ConcurrentHashMap<>();

  // Add heat map
  private final Map<Point, Double> heatMap = new ConcurrentHashMap<>();
  private long lastHeatDecay = System.currentTimeMillis();

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

    // Decay heat map periodically
    updateHeatMap();

    // Calculate base score
    double score = calculateBaseScore(agName, nextZone, info);

    // Add distance bonus from current position
    double distanceBonus = calculateDistanceBonus(agName, nextZone);

    // Get heat penalty
    double heatScore = getHeatScore(nextZone);

    // Add local area saturation penalty
    double localAreaPenalty = calculateLocalAreaPenalty(agName, nextZone);

    // Combine all factors
    return (
      (score + distanceBonus) * (1.0 - heatScore) * (1.0 - localAreaPenalty)
    );
  }

  private double calculateBaseScore(String agName, Point zone, ZoneInfo info) {
    Set<Point> agentZones = agentVisitedZones.computeIfAbsent(
      agName,
      k -> new HashSet<>()
    );

    if (!agentZones.contains(zone)) {
      return UNEXPLORED_BONUS;
    } else {
      return info.explorationScore * REVISIT_PENALTY;
    }
  }

  private void updateHeatMap() {
    long now = System.currentTimeMillis();
    if (now - lastHeatDecay > 1000) { // Decay every second
      heatMap.replaceAll((k, v) -> Math.max(MIN_HEAT, v * HEAT_DECAY_RATE));
      lastHeatDecay = now;
    }
  }

  public void recordVisit(String agName, Point position) {
    Point zone = getZone(position);
    ZoneInfo info = zoneMemory.computeIfAbsent(zone, k -> new ZoneInfo());
    info.visit(agName);

    agentVisitedZones.computeIfAbsent(agName, k -> new HashSet<>()).add(zone);

    // Update heat map
    updateHeatInRadius(zone);
  }

  private void updateHeatInRadius(Point center) {
    for (int dx = -HEAT_RADIUS; dx <= HEAT_RADIUS; dx++) {
      for (int dy = -HEAT_RADIUS; dy <= HEAT_RADIUS; dy++) {
        Point p = new Point(center.x + dx, center.y + dy);
        double distance = Math.sqrt(dx * dx + dy * dy);
        if (distance <= HEAT_RADIUS) {
          // Exponential heat decay with distance
          double heatValue = INITIAL_HEAT * Math.exp(-distance / HEAT_RADIUS);
          heatMap.merge(p, heatValue, (old, new_) -> Math.min(1.0, old + new_));
        }
      }
    }
  }

  private double getHeatScore(Point zone) {
    return heatMap.getOrDefault(zone, 0.0);
  }

  // Method to get heat map for visualization or debugging
  public Map<Point, Double> getHeatMap() {
    return new HashMap<>(heatMap);
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

  private double calculateDistanceBonus(String agName, Point targetZone) {
    Set<Point> visitedZones = agentVisitedZones.get(agName);
    if (visitedZones == null || visitedZones.isEmpty()) {
      return 0.0;
    }

    // Find average position of recently visited zones
    Point centroid = calculateCentroid(visitedZones);
    double distance = euclideanDistance(centroid, targetZone);

    // Bonus increases with distance from recent exploration center
    return Math.min(1.0, distance * DISTANCE_BONUS_FACTOR);
  }

  private Point calculateCentroid(Set<Point> points) {
    int sumX = 0, sumY = 0;
    for (Point p : points) {
      sumX += p.x;
      sumY += p.y;
    }
    return new Point(sumX / points.size(), sumY / points.size());
  }

  private double calculateLocalAreaPenalty(String agName, Point zone) {
    int localVisits = countLocalVisits(agName, zone);
    return Math.min(1.0, localVisits / (double) MAX_LOCAL_VISITS);
  }

  private int countLocalVisits(String agName, Point zone) {
    Set<Point> visitedZones = agentVisitedZones.get(agName);
    if (visitedZones == null) return 0;

    return (int) visitedZones
      .stream()
      .filter(p -> euclideanDistance(p, zone) <= LOCAL_AREA_RADIUS)
      .count();
  }

  private double euclideanDistance(Point p1, Point p2) {
    int dx = p1.x - p2.x;
    int dy = p1.y - p2.y;
    return Math.sqrt(dx * dx + dy * dy);
  }
}
