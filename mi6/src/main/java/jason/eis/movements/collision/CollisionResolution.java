package jason.eis.movements.collision;

public class CollisionResolution {
  private final String direction;
  private final String reason;

  public CollisionResolution(String direction, String reason) {
    this.direction = direction;
    this.reason = reason;
  }

  public String getDirection() {
    return direction;
  }

  public String getReason() {
    return reason;
  }

  public boolean isOscillation() {
    return "OSCILLATION".equals(reason);
  }

  public boolean isStuck() {
    return "STUCK".equals(reason);
  }
}
