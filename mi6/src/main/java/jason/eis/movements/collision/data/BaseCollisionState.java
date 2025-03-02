package jason.eis.movements.collision.data;

import jason.eis.Point;
import java.util.*;
import java.util.concurrent.ConcurrentHashMap;

public class BaseCollisionState {
  // Movement history constants
  public static final int MOVEMENT_HISTORY_SIZE = 9;
  private static final int MEMORY_SIZE = 5;

  // Core movement tracking
  private final Map<String, Deque<MovementRecord>> movementHistory = new ConcurrentHashMap<>();
  private final Map<String, Integer> agentSizes = new ConcurrentHashMap<>();
  private final Map<String, String> blockAttachments = new ConcurrentHashMap<>();

  // Agent state tracking
  private final Map<String, Deque<String>> lastMoves = new ConcurrentHashMap<>();
  private final Map<String, Point> lastPositions = new ConcurrentHashMap<>();
  private final Map<String, String> lastRecommendedDirections = new ConcurrentHashMap<>();

  // Boundary state tracking
  private final Map<String, Integer> boundaryAttempts = new ConcurrentHashMap<>();
  private final Map<String, Boolean> isOnBoundary = new ConcurrentHashMap<>();
  private final Map<String, String> lastBoundaryDirections = new ConcurrentHashMap<>();

  public void recordMovement(
    String agentId,
    Point position,
    String intendedDirection
  ) {
    // Record movement history
    Deque<MovementRecord> history = movementHistory.computeIfAbsent(
      agentId,
      k -> new LinkedList<>()
    );
    history.addFirst(new MovementRecord(position, intendedDirection));
    if (history.size() > MOVEMENT_HISTORY_SIZE) {
      history.removeLast();
    }

    // Update last moves
    Deque<String> moves = lastMoves.computeIfAbsent(
      agentId,
      k -> new ArrayDeque<>(MEMORY_SIZE)
    );
    if (moves.size() >= MEMORY_SIZE) {
      moves.removeLast();
    }
    moves.addFirst(intendedDirection);

    // Update position
    lastPositions.put(agentId, position);
    lastRecommendedDirections.put(agentId, intendedDirection);
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

  // Movement state getters
  public String getLastDirection(String agentId) {
    Deque<String> moves = lastMoves.get(agentId);
    return moves != null && !moves.isEmpty() ? moves.getFirst() : null;
  }

  public Point getLastPosition(String agentId) {
    return lastPositions.get(agentId);
  }

  public String getLastRecommendedDirection(String agentId) {
    return lastRecommendedDirections.get(agentId);
  }

  // Boundary state methods
  public void incrementBoundaryAttempts(String agentId) {
    boundaryAttempts.merge(agentId, 1, Integer::sum);
  }

  public int getBoundaryAttempts(String agentId) {
    return boundaryAttempts.getOrDefault(agentId, 0);
  }

  public void setOnBoundary(String agentId, boolean onBoundary) {
    isOnBoundary.put(agentId, onBoundary);
  }

  public boolean isOnBoundary(String agentId) {
    return isOnBoundary.getOrDefault(agentId, false);
  }

  public void setLastBoundaryDirection(String agentId, String direction) {
    lastBoundaryDirections.put(agentId, direction);
  }

  public String getLastBoundaryDirection(String agentId) {
    return lastBoundaryDirections.get(agentId);
  }

  public void resetAgentState(String agentId) {
    movementHistory.remove(agentId);
    agentSizes.remove(agentId);
    blockAttachments.remove(agentId);
    lastMoves.remove(agentId);
    lastPositions.remove(agentId);
    lastRecommendedDirections.remove(agentId);
    boundaryAttempts.remove(agentId);
    isOnBoundary.remove(agentId);
    lastBoundaryDirections.remove(agentId);
  }
}
