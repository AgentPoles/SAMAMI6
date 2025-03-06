package jason.eis;

public class Entity {

  public enum EntityType {
    DISPENSER,
    BLOCK,
    GOAL,
    AGENT,
    OBSTACLE,
  }

  private final String id;
  private final EntityType type;
  private final String details;
  private final Point position;
  private final Point relativePos;

  public Entity(
    String id,
    EntityType type,
    String details,
    Point position,
    Point relativePos
  ) {
    this.id = id;
    this.type = type;
    this.details = details;
    this.position = position;
    this.relativePos = relativePos;
  }

  public String getId() {
    return id;
  }

  public EntityType getType() {
    return type;
  }

  public String getDetails() {
    return details;
  }

  public Point getPosition() {
    return position;
  }

  public Point getRelativePos() {
    return relativePos;
  }

  @Override
  public String toString() {
    return String.format("%s(%s) at %s (%s)", type, id, position, details);
  }
}
