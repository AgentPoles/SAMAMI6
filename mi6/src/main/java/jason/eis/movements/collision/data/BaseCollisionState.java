package jason.eis.movements.collision.data;

import jason.eis.Point;
import java.util.*;
import java.util.concurrent.ConcurrentHashMap;

public class BaseCollisionState {
  // Movement history constants
  public static final int MOVEMENT_HISTORY_SIZE = 9;

  // Core movement tracking
  private final Map<String, Deque<MovementRecord>> movementHistory = new ConcurrentHashMap<>();
  private final Map<String, Integer> agentSizes = new ConcurrentHashMap<>();
  private final Map<String, String> blockAttachments = new ConcurrentHashMap<>();

  public void recordMovement(
    String agentId,
    Point position,
    String intendedDirection
  ) {
    Deque<MovementRecord> history = movementHistory.computeIfAbsent(
      agentId,
      k -> new LinkedList<>()
    );

    history.addFirst(new MovementRecord(position, intendedDirection));
    if (history.size() > MOVEMENT_HISTORY_SIZE) {
      history.removeLast();
    }
  }

  public void updateAgentState(
    String agentId,
    int size,
    String blockAttachment
  ) {
    agentSizes.put(agentId, size);
    if (blockAttachment != null) {
      blockAttachments.put(agentId, blockAttachment);
    } else {
      blockAttachments.remove(agentId);
    }
  }

  public List<MovementRecord> getMovementHistory(String agentId) {
    Deque<MovementRecord> history = movementHistory.get(agentId);
    return history != null ? new ArrayList<>(history) : new ArrayList<>();
  }

  public MovementRecord getLastMovement(String agentId) {
    Deque<MovementRecord> history = movementHistory.get(agentId);
    return history != null && !history.isEmpty() ? history.getFirst() : null;
  }

  public int getAgentSize(String agentId) {
    return agentSizes.getOrDefault(agentId, 1);
  }

  public String getBlockAttachment(String agentId) {
    return blockAttachments.get(agentId);
  }

  public void resetAgentState(String agentId) {
    movementHistory.remove(agentId);
    agentSizes.remove(agentId);
    blockAttachments.remove(agentId);
  }
}
