package jason.eis.movements;

import jason.eis.LocalMap;
import jason.eis.LocalMap.ObstacleInfo;
import jason.eis.Point;
import java.util.*;
import java.util.concurrent.ConcurrentHashMap;
import java.util.logging.Logger;
import java.util.stream.Collectors;

public class AgentCollisionHandler {
  private static final Logger logger = Logger.getLogger(
    AgentCollisionHandler.class.getName()
  );

  // Constants for collision handling
  private static final int CRITICAL_DISTANCE = 2; // Direct adjacency
  private static final int AWARENESS_DISTANCE = 3; // General awareness radius
  private static final int PATH_BLOCK_DISTANCE = 4; // Distance to check for path blocking
  private static final int MAX_WAIT_TURNS = 3; // Maximum turns to wait before finding alternative path
  private static final int STUCK_THRESHOLD = 3; // Number of failed moves before considering stuck
  private static final int STUCK_MEMORY_DURATION = 10000; // How long to remember stuck positions (ms)
  private static final int BACKOFF_DISTANCE = 3; // How far to try moving away when stuck
  private static final int PERSONAL_SPACE = 1;
  private static final int MAX_RECOVERY_ATTEMPTS = 3;

  // Track agent states
  private final Map<String, AgentState> agentStates = new ConcurrentHashMap<>();

  private static class AgentState {
    Point lastPosition;
    int stuckCount = 0;
    int recoveryAttempts = 0;
    int waitTurns = 0;
    Set<Point> stuckPositions = new HashSet<>();
    List<Point> moveHistory = new ArrayList<>();

    void recordMove(Point newPosition, String direction) {
      if (moveHistory.size() > 5) moveHistory.remove(0);
      moveHistory.add(newPosition);

      if (lastPosition != null && lastPosition.equals(newPosition)) {
        stuckCount++;
      } else {
        stuckCount = 0;
        recoveryAttempts = 0;
      }
      lastPosition = newPosition;
    }

    void recordFailure() {
      stuckCount++;
      if (lastPosition != null) {
        stuckPositions.add(lastPosition);
      }
    }

    void recordSuccess() {
      stuckCount = 0;
      recoveryAttempts = 0;
      waitTurns = 0;
    }

    List<Point> getMoveHistory() {
      return moveHistory;
    }

    boolean isStuck() {
      return stuckCount >= STUCK_THRESHOLD;
    }
  }

  public String resolveCollision(
    String agentName,
    Point currentPos,
    String intendedDirection,
    LocalMap map,
    boolean hasBlock
  ) {
    AgentState state = agentStates.computeIfAbsent(
      agentName,
      k -> new AgentState()
    );
    state.recordMove(currentPos, intendedDirection);

    // Check if we're in a known stuck position
    if (state.isStuck()) {
      return handleStuckSituation(state, currentPos, map, hasBlock);
    }

    // Normal collision resolution
    String resolvedDirection = resolveNormalCollision(
      state,
      agentName,
      currentPos,
      intendedDirection,
      map,
      hasBlock
    );

    // Update stuck detection
    if (
      resolvedDirection.equals("skip") ||
      !isMovePossible(currentPos, resolvedDirection, map)
    ) {
      state.recordFailure();

      // If we're now stuck, handle it
      if (state.isStuck()) {
        return handleStuckSituation(state, currentPos, map, hasBlock);
      }
    } else {
      state.recordSuccess();
    }

    return resolvedDirection;
  }

  private String handleStuckSituation(
    AgentState state,
    Point currentPos,
    LocalMap map,
    boolean hasBlock
  ) {
    // Try to find an escape direction
    List<String> availableDirections = getAvailableDirections(currentPos, map);
    if (availableDirections.isEmpty()) return null;

    // Try directions perpendicular to last movement
    List<String> perpendicularDirs = getPerpendicularDirections(
      state.getMoveHistory()
    );
    perpendicularDirs.retainAll(availableDirections);

    if (!perpendicularDirs.isEmpty()) {
      return perpendicularDirs.get(
        new Random().nextInt(perpendicularDirs.size())
      );
    }

    // Fallback to random available direction
    return availableDirections.get(
      new Random().nextInt(availableDirections.size())
    );
  }

  private String resolveNormalCollision(
    AgentState state,
    String agentName,
    Point currentPos,
    String intendedDirection,
    LocalMap map,
    boolean hasBlock
  ) {
    // Get nearby agents
    List<Point> nearbyAgents = getNearbyAgents(currentPos, map);

    // No nearby agents, continue with intended direction
    if (nearbyAgents.isEmpty()) {
      state.waitTurns = 0;
      return intendedDirection;
    }

    // Check for immediate collision risk
    Point nextPos = calculateNextPosition(currentPos, intendedDirection);
    if (isDirectCollisionRisk(nextPos, nearbyAgents)) {
      return handleImmediateCollision(
        state,
        currentPos,
        intendedDirection,
        nearbyAgents,
        map,
        hasBlock
      );
    }

    // Check for path blocking
    if (isPathBlocked(currentPos, intendedDirection, nearbyAgents)) {
      return handlePathBlocking(
        state,
        currentPos,
        intendedDirection,
        nearbyAgents,
        map
      );
    }

    // No collision risk, continue with intended direction
    state.waitTurns = 0;
    return intendedDirection;
  }

  private String handleImmediateCollision(
    AgentState state,
    Point currentPos,
    String intendedMove,
    List<Point> nearbyAgents,
    LocalMap map,
    boolean hasBlock
  ) {
    // If carrying a block, take more cautious approach
    if (hasBlock) {
      return findBestAvoidanceDirection(
        currentPos,
        getAvailableDirections(currentPos, map),
        nearbyAgents,
        map
      );
    }

    // Check if we should wait
    if (
      state.waitTurns < MAX_WAIT_TURNS &&
      shouldWaitForClear(currentPos, nearbyAgents)
    ) {
      state.waitTurns++;
      return "skip";
    }

    // Get perpendicular directions to avoid collision
    List<String> perpendicularDirs = getPerpendicularDirections(
      state.getMoveHistory()
    );
    String bestDirection = findBestAvoidanceDirection(
      currentPos,
      perpendicularDirs,
      nearbyAgents,
      map
    );

    if (bestDirection != null) {
      state.waitTurns = 0;
      return bestDirection;
    }

    // If no good avoidance direction, try opposite direction
    return getOppositeDirection(intendedMove);
  }

  private String handlePathBlocking(
    AgentState state,
    Point currentPos,
    String intendedDirection,
    List<Point> nearbyAgents,
    LocalMap map
  ) {
    // If we've been waiting too long, find alternative path
    if (state.waitTurns >= MAX_WAIT_TURNS) {
      state.waitTurns = 0;
      return findAlternativePath(
        currentPos,
        intendedDirection,
        nearbyAgents,
        map
      );
    }

    // Wait for path to clear
    state.waitTurns++;
    return "skip";
  }

  private String findAlternativePath(
    Point currentPos,
    String intendedDirection,
    List<Point> nearbyAgents,
    LocalMap map
  ) {
    // Get all possible directions except the blocked one
    List<String> alternatives = new ArrayList<>(
      Arrays.asList("n", "s", "e", "w")
    );
    alternatives.remove(intendedDirection);

    // Score each alternative direction
    Map<String, Double> scores = new HashMap<>();
    for (String dir : alternatives) {
      Point targetPos = calculateNextPosition(currentPos, dir);
      if (targetPos != null && !map.hasObstacle(targetPos)) {
        double score = scoreDirection(targetPos, nearbyAgents, map);
        scores.put(dir, score);
      }
    }

    // Return direction with highest score
    return scores
      .entrySet()
      .stream()
      .max(Map.Entry.comparingByValue())
      .map(Map.Entry::getKey)
      .orElse(getOppositeDirection(intendedDirection));
  }

  private double scoreDirection(
    Point targetPos,
    List<Point> nearbyAgents,
    LocalMap map
  ) {
    double score = 1.0;

    // Penalize directions that bring us closer to other agents
    for (Point agentPos : nearbyAgents) {
      double distance = euclideanDistance(targetPos, agentPos);
      if (distance < AWARENESS_DISTANCE) {
        score *= (distance / AWARENESS_DISTANCE);
      }
    }

    // Bonus for directions that lead to open space
    int openSpaces = countOpenSpaces(targetPos, map);
    score *= (1.0 + (openSpaces / 8.0));

    return score;
  }

  private int countOpenSpaces(Point pos, LocalMap map) {
    int count = 0;
    for (int dx = -1; dx <= 1; dx++) {
      for (int dy = -1; dy <= 1; dy++) {
        if (dx == 0 && dy == 0) continue;
        Point checkPos = new Point(pos.x + dx, pos.y + dy);
        if (!map.hasObstacle(checkPos)) {
          count++;
        }
      }
    }
    return count;
  }

  // Helper methods
  private List<Point> getNearbyAgents(Point currentPos, LocalMap map) {
    return map
      .getDynamicObstacles()
      .values()
      .stream()
      .filter(
        obs ->
          obs.isDynamic() &&
          euclideanDistance(currentPos, obs.getPosition()) <= AWARENESS_DISTANCE
      )
      .map(obs -> obs.getPosition())
      .collect(Collectors.toList());
  }

  private boolean isDirectCollisionRisk(
    Point nextPos,
    List<Point> nearbyAgents
  ) {
    return nearbyAgents
      .stream()
      .anyMatch(
        agentPos -> euclideanDistance(nextPos, agentPos) <= CRITICAL_DISTANCE
      );
  }

  private boolean isPathBlocked(
    Point currentPos,
    String direction,
    List<Point> nearbyAgents
  ) {
    Point targetPos = calculateNextPosition(currentPos, direction);
    if (targetPos == null) return true;

    return nearbyAgents
      .stream()
      .anyMatch(agentPos -> isInPath(currentPos, targetPos, agentPos));
  }

  private boolean isInPath(Point start, Point end, Point check) {
    double distance = pointToLineDistance(check, start, end);
    return distance < PATH_BLOCK_DISTANCE;
  }

  private double pointToLineDistance(
    Point point,
    Point lineStart,
    Point lineEnd
  ) {
    double normalLength = euclideanDistance(lineStart, lineEnd);
    if (normalLength == 0) return euclideanDistance(point, lineStart);

    double t =
      (
        (point.x - lineStart.x) *
        (lineEnd.x - lineStart.x) +
        (point.y - lineStart.y) *
        (lineEnd.y - lineStart.y)
      ) /
      (normalLength * normalLength);

    if (t < 0) return euclideanDistance(point, lineStart);
    if (t > 1) return euclideanDistance(point, lineEnd);

    return euclideanDistance(
      point,
      new Point(
        (int) (lineStart.x + t * (lineEnd.x - lineStart.x)),
        (int) (lineStart.y + t * (lineEnd.y - lineStart.y))
      )
    );
  }

  private double euclideanDistance(Point p1, Point p2) {
    int dx = p2.x - p1.x;
    int dy = p2.y - p1.y;
    return Math.sqrt(dx * dx + dy * dy);
  }

  private Point calculateNextPosition(Point current, String direction) {
    if (direction == null || direction.equals("skip")) return current;

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

  private List<String> getPerpendicularDirections(List<Point> moveHistory) {
    if (moveHistory.size() < 2) {
      return Arrays.asList("n", "s", "e", "w");
    }

    Point last = moveHistory.get(moveHistory.size() - 1);
    Point prev = moveHistory.get(moveHistory.size() - 2);

    if (last.x != prev.x) {
      return Arrays.asList("n", "s");
    }
    if (last.y != prev.y) {
      return Arrays.asList("e", "w");
    }

    return Arrays.asList("n", "s", "e", "w");
  }

  private String getOppositeDirection(String direction) {
    switch (direction) {
      case "n":
        return "s";
      case "s":
        return "n";
      case "e":
        return "w";
      case "w":
        return "e";
      default:
        return direction;
    }
  }

  public void recordSuccessfulMove(String agentName, Point newPosition) {
    AgentState state = agentStates.get(agentName);
    if (state != null) {
      state.recordMove(newPosition, null);
    }
  }

  public boolean isInPersonalSpace(Point position, Point blockCarrierPosition) {
    return (
      Math.abs(position.x - blockCarrierPosition.x) <= PERSONAL_SPACE &&
      Math.abs(position.y - blockCarrierPosition.y) <= PERSONAL_SPACE
    );
  }

  private List<String> getAvailableDirections(Point pos, LocalMap map) {
    List<String> directions = Arrays.asList("n", "s", "e", "w");
    return directions
      .stream()
      .filter(dir -> isMovePossible(pos, dir, map))
      .collect(Collectors.toList());
  }

  private boolean isMovePossible(Point pos, String direction, LocalMap map) {
    Point next = calculateNextPosition(pos, direction);
    return !map.hasObstacle(next) && !map.isOutOfBounds(next);
  }

  private boolean shouldWaitForClear(
    Point currentPos,
    List<Point> nearbyAgents
  ) {
    // Implement waiting logic based on nearby agent positions and movements
    return nearbyAgents
      .stream()
      .anyMatch(agent -> getManhattanDistance(currentPos, agent) == 1);
  }

  private int getManhattanDistance(Point p1, Point p2) {
    return Math.abs(p1.x - p2.x) + Math.abs(p1.y - p2.y);
  }

  private String findBestAvoidanceDirection(
    Point currentPos,
    List<String> availableDirections,
    List<Point> nearbyAgents,
    LocalMap map
  ) {
    if (availableDirections.isEmpty()) return null;

    // Score each direction based on distance from other agents
    Map<String, Double> directionScores = new HashMap<>();

    for (String direction : availableDirections) {
      Point nextPos = calculateNextPosition(currentPos, direction);
      double score = evaluatePosition(nextPos, nearbyAgents, map);
      directionScores.put(direction, score);
    }

    return directionScores
      .entrySet()
      .stream()
      .max(Map.Entry.comparingByValue())
      .map(Map.Entry::getKey)
      .orElse(null);
  }

  private double evaluatePosition(
    Point pos,
    List<Point> nearbyAgents,
    LocalMap map
  ) {
    double score = 1.0;

    // Penalize positions close to other agents
    for (Point agent : nearbyAgents) {
      int distance = getManhattanDistance(pos, agent);
      score *= (distance / (double) CRITICAL_DISTANCE);
    }

    // Bonus for positions away from obstacles
    if (!map.hasObstacle(pos)) {
      score *= 1.2;
    }

    return score;
  }
}
