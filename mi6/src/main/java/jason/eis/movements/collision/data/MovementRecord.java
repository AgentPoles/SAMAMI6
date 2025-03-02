package jason.eis.movements.collision.data;

import jason.eis.Point;

public class MovementRecord {
  private final Point position;
  private final String intendedDirection;
  private final long timestamp;

  public MovementRecord(Point position, String intendedDirection) {
    this.position = position;
    this.intendedDirection = intendedDirection;
    this.timestamp = System.currentTimeMillis();
  }

  public Point getPosition() {
    return position;
  }

  public String getIntendedDirection() {
    return intendedDirection;
  }

  public long getTimestamp() {
    return timestamp;
  }
}
