package jason.eis.movements;

import jason.eis.Point;
import java.util.*;

public class ObstacleMemory {
  private final Map<Point, ObstacleRecord> obstacles = new HashMap<>();
  private static final long MEMORY_DURATION = 5000; // 5 seconds

  private static class ObstacleRecord {
    final String type;
    long lastSeen;
    Point velocity;

    ObstacleRecord(String type) {
      this.type = type;
      this.lastSeen = System.currentTimeMillis();
    }

    void updateSeen(Point newPos, Point oldPos) {
      if (oldPos != null) {
        velocity = new Point(newPos.x - oldPos.x, newPos.y - oldPos.y);
      }
      lastSeen = System.currentTimeMillis();
    }

    boolean isStale() {
      return System.currentTimeMillis() - lastSeen > MEMORY_DURATION;
    }

    Point predictPosition(int steps) {
      if (velocity == null) return null;
      return new Point(velocity.x * steps, velocity.y * steps);
    }
  }

  public void updateObstacle(Point position, String type) {
    ObstacleRecord record = obstacles.get(position);
    if (record != null) {
      record.updateSeen(position, record.predictPosition(1));
    } else {
      obstacles.put(position, new ObstacleRecord(type));
    }
    cleanStaleObstacles();
  }

  public Set<Point> getPredictedPositions(int steps) {
    Set<Point> predictions = new HashSet<>();
    for (Map.Entry<Point, ObstacleRecord> entry : obstacles.entrySet()) {
      predictions.add(entry.getKey()); // Current position
      Point predicted = entry.getValue().predictPosition(steps);
      if (predicted != null) {
        predictions.add(predicted);
      }
    }
    return predictions;
  }

  private void cleanStaleObstacles() {
    obstacles.entrySet().removeIf(entry -> entry.getValue().isStale());
  }

  public boolean hasNearbyObstacles(Point position, double radius) {
    return obstacles
      .keySet()
      .stream()
      .anyMatch(p -> euclideanDistance(p, position) <= radius);
  }

  private double euclideanDistance(Point p1, Point p2) {
    double dx = p1.x - p2.x;
    double dy = p1.y - p2.y;
    return Math.sqrt(dx * dx + dy * dy);
  }
}
