package jason.eis.movements.collision.data;

import jason.eis.Point;
import java.util.*;
import java.util.concurrent.ConcurrentHashMap;

public class StuckState {
  // Stuck detection constants
  public static final int STUCK_THRESHOLD = 3;
  public static final long STUCK_TIMEOUT = 5000; // 5 seconds

  private final Map<String, Integer> stuckCounter = new ConcurrentHashMap<>();
  private final Map<String, Long> stuckStartTimes = new ConcurrentHashMap<>();
  private final Map<String, Set<String>> triedDirections = new ConcurrentHashMap<>();

  public void incrementStuck(String agentId, String attemptedDirection) {
    if (!stuckCounter.containsKey(agentId)) {
      stuckStartTimes.put(agentId, System.currentTimeMillis());
      triedDirections.put(agentId, new HashSet<>());
    }
    stuckCounter.merge(agentId, 1, Integer::sum);
    if (attemptedDirection != null) {
      triedDirections.get(agentId).add(attemptedDirection);
    }
  }

  public boolean isStuck(String agentId) {
    return stuckCounter.getOrDefault(agentId, 0) >= STUCK_THRESHOLD;
  }

  public Set<String> getTriedDirections(String agentId) {
    return triedDirections.getOrDefault(agentId, new HashSet<>());
  }

  public void resetStuckState(String agentId) {
    stuckCounter.remove(agentId);
    stuckStartTimes.remove(agentId);
    triedDirections.remove(agentId);
  }

  public boolean isStuckTimeout(String agentId) {
    Long startTime = stuckStartTimes.get(agentId);
    return (
      startTime != null &&
      System.currentTimeMillis() - startTime > STUCK_TIMEOUT
    );
  }
}
