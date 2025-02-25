package jason.eis;

public class Point {
  public final int x, y;
  private final int hashCode;

  public Point(int x, int y) {
    this.x = x;
    this.y = y;
    this.hashCode = 31 * x + y;
  }

  @Override
  public int hashCode() {
    return hashCode;
  }

  @Override
  public boolean equals(Object o) {
    if (this == o) return true;
    if (!(o instanceof Point)) return false;
    Point p = (Point) o;
    return x == p.x && y == p.y;
  }
}
