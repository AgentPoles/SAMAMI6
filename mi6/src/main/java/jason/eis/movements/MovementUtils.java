package jason.eis.movements;

import jason.eis.LocalMap;
import jason.eis.Point;
import java.util.Map;
import java.util.logging.Logger;

public class MovementUtils {
  private static final Logger logger = Logger.getLogger(
    MovementUtils.class.getName()
  );

  public static boolean wouldHitBoundary(LocalMap map, String direction) {
    if (map == null || direction == null) return false;

    Point currentPos = map.getCurrentPosition();
    Map<String, Point> boundaries = map.getConfirmedBoundariesPositions();

    if (currentPos == null || boundaries.isEmpty()) return false;

    // Check if we have a confirmed boundary in this direction
    Point boundaryPoint = boundaries.get(direction);
    if (boundaryPoint == null) return false;

    // Calculate relative position after move
    Point targetPos = calculateNextPosition(currentPos, direction);

    // Check if target position matches known boundary
    return targetPos.equals(boundaryPoint);
  }

  public static Point calculateNextPosition(Point current, String direction) {
    if (current == null || direction == null) return null;

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
