package helpermodels;

import jason.asSemantics.DefaultInternalAction;
import jason.asSemantics.TransitionSystem;
import jason.asSemantics.Unifier;
import jason.asSyntax.*;
import jason.eis.MI6Model;
import jason.eis.Point;
import jason.eis.movements.PlannedMovement;
import jason.eis.movements.RandomMovement;
import jason.eis.movements.Search;
import java.util.ArrayList;
import java.util.List;
import java.util.Map;
import java.util.concurrent.*;
import java.util.logging.Level;
import java.util.logging.Logger;

public class RequestGuidance extends DefaultInternalAction {
  private static final Logger logger = Logger.getLogger(
    RequestGuidance.class.getName()
  );
  private static final int PATH_TIMEOUT = Integer.MAX_VALUE; // effectively no timeout
  private static final int RECALCULATION_COOLDOWN = 0; // effectively no cooldown
  private static final boolean DEBUG = true; // Toggle debug logging

  private static class PathState {
    List<String> currentPath;
    List<String> pathHistory;
    long lastFailedAttempt;
    Point targetPosition;
    String targetType;

    PathState() {
      this.pathHistory = new ArrayList<>();
      this.lastFailedAttempt = 0;
    }

    void recordPath(List<String> path) {
      if (this.currentPath != null && !this.currentPath.isEmpty()) {
        pathHistory.addAll(this.currentPath);
      }
      this.currentPath = path;
    }

    boolean canRetryPathfinding() {
      return (
        System.currentTimeMillis() - lastFailedAttempt > RECALCULATION_COOLDOWN
      );
    }

    void recordFailedAttempt() {
      this.lastFailedAttempt = System.currentTimeMillis();
    }
  }

  private static final Map<String, PathState> agentPaths = new ConcurrentHashMap<>();

  @Override
  public Object execute(TransitionSystem ts, Unifier un, Term[] terms)
    throws Exception {
    try {
      String agName = ts.getUserAgArch().getAgName();
      MI6Model model = MI6Model.getInstance();

      if (DEBUG) logger.info(
        String.format("[%s] Starting RequestGuidance execution", agName)
      );

      // Parse parameters
      String targetTypeStr = ((Atom) terms[0]).getFunctor().toUpperCase();

      // Convert ASL atoms to enum values in a single assignment
      final Search.TargetType targetType = convertTargetType(
        targetTypeStr,
        agName
      );

      int size = (int) ((NumberTerm) terms[1]).solve();
      Term outputTerm = terms[terms.length - 1];

      PathState pathState = agentPaths.computeIfAbsent(
        agName,
        k -> new PathState()
      );

      // Get current position and map
      Point currentPos = model.getAgentMap(agName).getCurrentPosition();
      PlannedMovement plannedMovement = model.getPlannedMovement();

      // If we failed recently, use random movement
      if (!pathState.canRetryPathfinding()) {
        if (DEBUG) {
          long waitTime =
            RECALCULATION_COOLDOWN -
            (System.currentTimeMillis() - pathState.lastFailedAttempt);
          logger.info(
            String.format(
              "[%s] In cooldown period. %dms remaining before retry",
              agName,
              waitTime
            )
          );
        }
        return fallbackToRandomMovement(agName, un, outputTerm);
      }

      try {
        CompletableFuture<Search.PathResult> pathFuture = CompletableFuture.supplyAsync(
          () -> {
            // Find nearest target
            Point target = plannedMovement.findNearestTarget(
              model.getAgentMap(agName),
              currentPos,
              targetType
            );

            if (target == null) return null;

            // Calculate path to target
            return plannedMovement.calculatePath(
              model.getAgentMap(agName),
              currentPos,
              target,
              targetType
            );
          }
        );

        Search.PathResult pathResult = pathFuture.get();

        if (
          pathResult != null &&
          pathResult.success &&
          !pathResult.directions.isEmpty()
        ) {
          if (DEBUG) {
            logger.info(
              String.format(
                "[%s] Path found! Length: %d, First step: %s",
                agName,
                pathResult.directions.size(),
                pathResult.directions.get(0)
              )
            );
          }

          pathState.recordPath(new ArrayList<>(pathResult.directions));
          pathState.targetPosition =
            pathResult.points.get(pathResult.points.size() - 1);
          pathState.targetType = targetType.toString();

          String nextDirection = pathResult.directions.get(0);
          pathState.currentPath.remove(0);
          return returnSingleDirection(nextDirection, un, outputTerm);
        }
      } catch (Exception e) {
        if (DEBUG) logger.log(
          Level.WARNING,
          String.format("[%s] Path calculation failed", agName),
          e
        );
      }

      // Path calculation failed
      pathState.recordFailedAttempt();
      if (DEBUG) logger.info(
        String.format("[%s] Falling back to random movement", agName)
      );
      return fallbackToRandomMovement(agName, un, outputTerm);
    } catch (Exception e) {
      logger.log(Level.SEVERE, "Error in RequestGuidance execution", e);
      throw e;
    }
  }

  private Object fallbackToRandomMovement(
    String agName,
    Unifier un,
    Term outputTerm
  ) {
    MI6Model model = MI6Model.getInstance();
    RandomMovement randomMovement = model.getRandomMovement();
    String randomDir = randomMovement.getNextDirection(agName);
    if (randomDir == null) randomDir = "n";

    if (DEBUG) {
      logger.info(
        String.format(
          "[%s] Using random movement: direction=%s",
          agName,
          randomDir
        )
      );
    }

    return returnSingleDirection(randomDir, un, outputTerm);
  }

  private Object returnSingleDirection(
    String direction,
    Unifier un,
    Term outputTerm
  ) {
    if (DEBUG) {
      logger.info(String.format("Returning single direction: %s", direction));
    }

    // Return the direction as a single atom instead of a list
    return un.unifies(new Atom(direction), outputTerm);
  }

  // Helper method to convert string to enum
  private Search.TargetType convertTargetType(
    String targetTypeStr,
    String agName
  ) {
    try {
      switch (targetTypeStr) {
        case "GOAL":
          return Search.TargetType.GOAL;
        case "DISPENSER":
          return Search.TargetType.DISPENSER;
        case "BLOCK":
          return Search.TargetType.BLOCK;
        case "TARGET": // Handle legacy "target" atom if needed
          return Search.TargetType.DISPENSER;
        default:
          logger.warning(
            "[" +
            agName +
            "] Unknown target type: " +
            targetTypeStr +
            ", defaulting to DISPENSER"
          );
          return Search.TargetType.DISPENSER;
      }
    } catch (Exception e) {
      logger.warning(
        "[" +
        agName +
        "] Error converting target type: " +
        targetTypeStr +
        ", defaulting to DISPENSER"
      );
      return Search.TargetType.DISPENSER;
    }
  }
}
