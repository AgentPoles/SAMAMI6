package jason.eis;

import java.util.*;

public class Zone {
  public final int x, y;
  private final int hashCode;
  private final Set<Point> exploredPoints;

  public Zone(int x, int y) {
    this.x = x;
    this.y = y;
    this.hashCode = 31 * x + y;
    this.exploredPoints = new HashSet<>();
  }

  public void recordExploration(Point point) {
    exploredPoints.add(point);
  }

  public boolean isExplored(Point point) {
    return exploredPoints.contains(point);
  }

  public int getExplorationCount() {
    return exploredPoints.size();
  }

  @Override
  public int hashCode() {
    return hashCode;
  }

  @Override
  public boolean equals(Object o) {
    if (this == o) return true;
    if (!(o instanceof Zone)) return false;
    Zone z = (Zone) o;
    return x == z.x && y == z.y;
  }

  @Override
  public String toString() {
    return String.format("Zone(%d,%d)", x, y);
  }
}
