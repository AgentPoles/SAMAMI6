package jason.eis;

import java.util.Objects;

public class Point {
  public final int x, y;
  private final int hashCode;

  public Point(int x, int y) {
    this.x = x;
    this.y = y;
    this.hashCode = Objects.hash(x, y);
  }

  @Override
  public int hashCode() {
    return hashCode;
  }

  @Override
  public boolean equals(Object o) {
    if (this == o) return true;
    if (o == null || getClass() != o.getClass()) return false;
    Point point = (Point) o;
    return x == point.x && y == point.y;
  }

  @Override
  public String toString() {
    return String.format("(%d,%d)", x, y);
  }
}
