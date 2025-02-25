package jason.eis;

import eis.EnvironmentInterfaceStandard;
import eis.iilang.*;
import jason.asSyntax.*;
import jason.environment.Environment;
import java.util.*;
import java.util.logging.Level;
import java.util.logging.Logger;

public class MI6Model {
  private Logger logger = Logger.getLogger(
    "MI6Model." + MI6Model.class.getName()
  );
  private EnvironmentInterfaceStandard ei;
  private Random random = new Random();
  private static final String[] DIRECTIONS = { "n", "s", "e", "w" };

  // Zone management
  private Map<String, Point> agentZones = new HashMap<>();
  private Map<String, Integer> zoneSteps = new HashMap<>();
  private static final int ZONE_SIZE = 10;
  private static final int STEPS_BEFORE_ZONE_CHANGE = 15;

  // Target management
  private Map<String, Point> currentTargets = new HashMap<>();
  private Map<String, Integer> targetAttempts = new HashMap<>();
  private Map<String, List<Point>> recentTargets = new HashMap<>();
  private static final int MAX_TARGET_ATTEMPTS = 10;
  private static final int MAX_RECENT_TARGETS = 3;

  // Movement tracking
  private Map<String, MovementHistory> agentMovement = new HashMap<>();
  private static final int STUCK_THRESHOLD = 3;
  private static final int CLOSE_TARGET_DISTANCE = 5; // Consider target "close" within 5 steps

  private static class Point {
    int x, y;

    Point(int x, int y) {
      this.x = x;
      this.y = y;
    }

    @Override
    public boolean equals(Object o) {
      if (this == o) return true;
      if (o == null || getClass() != o.getClass()) return false;
      Point point = (Point) o;
      return x == point.x && y == point.y;
    }

    @Override
    public int hashCode() {
      return Objects.hash(x, y);
    }
  }

  private class MovementHistory {
    Point lastPosition;
    int stuckCount;
    String lastFailure;

    MovementHistory() {
      lastPosition = new Point(0, 0);
      stuckCount = 0;
      lastFailure = null;
    }
  }

  public MI6Model(EnvironmentInterfaceStandard ei) {
    this.ei = ei;
  }

  public boolean moveTowards(String agName, String direction) {
    try {
      ei.performAction(agName, new Action("move", new Identifier(direction)));
      // Reset failure tracking on successful move
      MovementHistory history = agentMovement.get(agName);
      if (history != null) {
        history.lastFailure = null;
      }
      return true;
    } catch (Exception e) {
      // Track the failure type
      MovementHistory history = agentMovement.computeIfAbsent(
        agName,
        k -> new MovementHistory()
      );
      String message = e.getMessage().toLowerCase();
      if (message.contains("failed_forbidden")) {
        history.lastFailure = "failed_forbidden";
      } else if (message.contains("failed_path")) {
        history.lastFailure = "failed_path";
      }
      logger.log(Level.SEVERE, "Error in moveTowards", e);
      return false;
    }
  }

  public boolean requestBlock(String agName, String direction) {
    try {
      ei.performAction(
        agName,
        new Action("request", new Identifier(direction))
      );
      return true;
    } catch (Exception e) {
      logger.log(Level.SEVERE, "Error in requestBlock", e);
      return false;
    }
  }

  public boolean attachBlock(String agName, String direction) {
    try {
      ei.performAction(agName, new Action("attach", new Identifier(direction)));
      return true;
    } catch (Exception e) {
      logger.log(Level.SEVERE, "Error in attachBlock", e);
      return false;
    }
  }

  public String chooseBestDirection(String agName, int targetX, int targetY) {
    try {
      // First check if we're stuck
      Point currentPos = getCurrentPosition(agName);
      MovementHistory history = agentMovement.computeIfAbsent(
        agName,
        k -> new MovementHistory()
      );

      if (currentPos.equals(history.lastPosition)) {
        history.stuckCount++;
        if (history.stuckCount >= STUCK_THRESHOLD) {
          String escapeDir = handleStuckAgent(agName, history.lastFailure);
          if (escapeDir != null) return escapeDir;
        }
      } else {
        history.stuckCount = 0;
        history.lastPosition = currentPos;
      }

      // If we have a target, determine if it's close
      if (targetX != 0 || targetY != 0) {
        boolean isClose =
          Math.abs(targetX) <= CLOSE_TARGET_DISTANCE &&
          Math.abs(targetY) <= CLOSE_TARGET_DISTANCE;

        // If we're aligned with a close target, prioritize direct movement
        if (isClose) {
          if (targetX == 0) {
            if (
              targetY < 0 && checkSafeDirection(agName, "n") != null
            ) return "n";
            if (
              targetY > 0 && checkSafeDirection(agName, "s") != null
            ) return "s";
          }
          if (targetY == 0) {
            if (
              targetX < 0 && checkSafeDirection(agName, "w") != null
            ) return "w";
            if (
              targetX > 0 && checkSafeDirection(agName, "e") != null
            ) return "e";
          }
        }

        // For close targets, prefer moving towards target even if not aligned
        if (isClose) {
          Map<String, Double> risks = evaluateRisks(agName, targetX, targetY);
          String bestDir = null;
          double bestRisk = Double.MAX_VALUE;

          // Find safest direction that moves closer to target
          for (Map.Entry<String, Double> entry : risks.entrySet()) {
            if (entry.getValue() < 1000.0 && entry.getValue() < bestRisk) {
              String dir = entry.getKey();
              boolean movesCloser =
                (targetX < 0 && dir.equals("w")) ||
                (targetX > 0 && dir.equals("e")) ||
                (targetY < 0 && dir.equals("n")) ||
                (targetY > 0 && dir.equals("s"));

              if (movesCloser) {
                bestDir = dir;
                bestRisk = entry.getValue();
              }
            }
          }

          if (bestDir != null) return bestDir;
        }
      }

      // If no close target or couldn't move directly, use alternative strategy
      return chooseAlternativeDirection(agName, targetX, targetY);
    } catch (Exception e) {
      logger.log(Level.SEVERE, "Error in chooseBestDirection", e);
      return DIRECTIONS[random.nextInt(DIRECTIONS.length)];
    }
  }

  private String chooseAlternativeDirection(
    String agName,
    int targetX,
    int targetY
  ) {
    try {
      Map<String, Double> risks = evaluateRisks(agName, targetX, targetY);
      List<String> safeDirections = new ArrayList<>();

      for (Map.Entry<String, Double> entry : risks.entrySet()) {
        if (entry.getValue() < 1000.0) {
          safeDirections.add(entry.getKey());
        }
      }

      if (safeDirections.isEmpty()) {
        return DIRECTIONS[random.nextInt(DIRECTIONS.length)];
      }

      // If we have a target but it's not close, prefer general direction
      if (targetX != 0 || targetY != 0) {
        if (Math.abs(targetX) > Math.abs(targetY)) {
          String preferredDir = targetX > 0 ? "e" : "w";
          if (safeDirections.contains(preferredDir)) return preferredDir;
        } else {
          String preferredDir = targetY > 0 ? "s" : "n";
          if (safeDirections.contains(preferredDir)) return preferredDir;
        }
      }

      // If no target, use zone-based exploration
      return handleZoneExploration(agName, safeDirections);
    } catch (Exception e) {
      logger.log(Level.SEVERE, "Error in chooseAlternativeDirection", e);
      return DIRECTIONS[random.nextInt(DIRECTIONS.length)];
    }
  }

  private String handleZoneExploration(
    String agName,
    List<String> safeDirections
  ) {
    Point zone = agentZones.computeIfAbsent(agName, k -> new Point(0, 0));
    int steps = zoneSteps.computeIfAbsent(agName, k -> 0);

    if (steps > STEPS_BEFORE_ZONE_CHANGE) {
      zone.x += (random.nextBoolean() ? ZONE_SIZE : -ZONE_SIZE);
      zone.y += (random.nextBoolean() ? ZONE_SIZE : -ZONE_SIZE);
      zoneSteps.put(agName, 0);
    } else {
      zoneSteps.put(agName, steps + 1);
    }

    String zoneDir = getDirectionTowardsZone(agName, zone);
    if (safeDirections.contains(zoneDir)) {
      return zoneDir;
    }

    return safeDirections.get(random.nextInt(safeDirections.size()));
  }

  private String checkSafeDirection(String agName, String preferredDir)
    throws Exception {
    // Check if preferred direction is safe
    Map<String, Double> risks = evaluateRisks(agName, 0, 0);
    if (risks.get(preferredDir) < 1000.0) {
      return preferredDir;
    }

    // If not safe, return null to fall back to other movement strategies
    return null;
  }

  private String getDirectionTowardsZone(String agName, Point zone) {
    try {
      // Get current position
      Map<String, Collection<Percept>> percepts = ei.getAllPercepts(agName);
      int currentX = 0, currentY = 0;

      for (Collection<Percept> perceptList : percepts.values()) {
        for (Percept p : perceptList) {
          if (p.getName().equals("position")) {
            Parameter[] params = p.getParameters().toArray(new Parameter[0]);
            currentX = ((Numeral) params[0]).getValue().intValue();
            currentY = ((Numeral) params[1]).getValue().intValue();
            break;
          }
        }
      }

      // Calculate direction towards zone center
      int dx = zone.x - currentX;
      int dy = zone.y - currentY;

      if (Math.abs(dx) > Math.abs(dy)) {
        return dx > 0 ? "e" : "w";
      } else {
        return dy > 0 ? "s" : "n";
      }
    } catch (Exception e) {
      logger.warning("Could not get position, using random direction");
      return DIRECTIONS[random.nextInt(DIRECTIONS.length)];
    }
  }

  private Map<String, Double> evaluateRisks(
    String agName,
    int targetX,
    int targetY
  )
    throws Exception {
    // Initialize risks
    Map<String, Double> risks = new HashMap<>();
    risks.put("n", 1.0);
    risks.put("s", 1.0);
    risks.put("e", 1.0);
    risks.put("w", 1.0);

    // Check for obstacles and other agents
    Map<String, Collection<Percept>> percepts = ei.getAllPercepts(agName);
    for (Collection<Percept> perceptList : percepts.values()) {
      for (Percept p : perceptList) {
        if (p.getName().equals("thing")) {
          Parameter[] params = p.getParameters().toArray(new Parameter[0]);
          int x = ((Numeral) params[0]).getValue().intValue();
          int y = ((Numeral) params[1]).getValue().intValue();
          String type = ((Identifier) params[2]).getValue();

          // Mark directions with obstacles as very high risk
          if (type.equals("obstacle") || type.equals("entity")) {
            if (x == 0 && y == -1) risks.put("n", 1000.0);
            if (x == 0 && y == 1) risks.put("s", 1000.0);
            if (x == 1 && y == 0) risks.put("e", 1000.0);
            if (x == -1 && y == 0) risks.put("w", 1000.0);
          }
        }
      }
    }

    // Adjust risks based on target direction
    if (targetX > 0) risks.put("e", Math.min(risks.get("e"), 0.5));
    if (targetX < 0) risks.put("w", Math.min(risks.get("w"), 0.5));
    if (targetY > 0) risks.put("s", Math.min(risks.get("s"), 0.5));
    if (targetY < 0) risks.put("n", Math.min(risks.get("n"), 0.5));

    return risks;
  }

  private Point getCurrentPosition(String agName) throws Exception {
    Map<String, Collection<Percept>> percepts = ei.getAllPercepts(agName);
    for (Collection<Percept> perceptList : percepts.values()) {
      for (Percept p : perceptList) {
        if (p.getName().equals("position")) {
          Parameter[] params = p.getParameters().toArray(new Parameter[0]);
          return new Point(
            ((Numeral) params[0]).getValue().intValue(),
            ((Numeral) params[1]).getValue().intValue()
          );
        }
      }
    }
    throw new Exception("Position not found");
  }

  private String handleStuckAgent(String agName, String lastFailure)
    throws Exception {
    Map<String, Double> risks = evaluateRisks(agName, 0, 0);
    List<String> safeDirections = new ArrayList<>();

    for (Map.Entry<String, Double> entry : risks.entrySet()) {
      if (entry.getValue() < 1000.0) {
        safeDirections.add(entry.getKey());
      }
    }

    if (!safeDirections.isEmpty()) {
      // If we hit a boundary (failed_forbidden), prefer moving away from it
      if ("failed_forbidden".equals(lastFailure)) {
        String[] opposites = { "s", "n", "w", "e" };
        Map<String, String> oppositeDir = new HashMap<>();
        for (int i = 0; i < DIRECTIONS.length; i++) {
          oppositeDir.put(DIRECTIONS[i], opposites[i]);
        }

        // Try to move in the opposite direction of the boundary
        String opposite = oppositeDir.get(getLastAttemptedDirection(agName));
        if (safeDirections.contains(opposite)) {
          return opposite;
        }
      }

      // If we're blocked by an obstacle/agent (failed_path), try perpendicular directions
      if ("failed_path".equals(lastFailure)) {
        String lastDir = getLastAttemptedDirection(agName);
        if ("n".equals(lastDir) || "s".equals(lastDir)) {
          if (safeDirections.contains("e")) return "e";
          if (safeDirections.contains("w")) return "w";
        } else {
          if (safeDirections.contains("n")) return "n";
          if (safeDirections.contains("s")) return "s";
        }
      }

      // If all else fails, pick a random safe direction
      return safeDirections.get(random.nextInt(safeDirections.size()));
    }

    return null; // Let the main logic handle it if we couldn't find a solution
  }

  private String getLastAttemptedDirection(String agName) {
    // Get the movement history for this agent
    MovementHistory history = agentMovement.get(agName);
    if (history != null && history.lastFailure != null) {
      // If we have a history and last failure, return the last direction
      Point currentPos = history.lastPosition;
      try {
        Point newPos = getCurrentPosition(agName);
        // Compare positions to determine attempted direction
        if (newPos.x > currentPos.x) return "e";
        if (newPos.x < currentPos.x) return "w";
        if (newPos.y > currentPos.y) return "s";
        if (newPos.y < currentPos.y) return "n";
      } catch (Exception e) {
        logger.warning("Could not determine last direction");
      }
    }
    // Default to north if we can't determine direction
    return "n";
  }
}
