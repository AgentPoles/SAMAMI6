package helpermodels;

import jason.asSemantics.DefaultInternalAction;
import jason.asSemantics.TransitionSystem;
import jason.asSemantics.Unifier;
import jason.asSyntax.*;
import jason.eis.LocalMap;
import jason.eis.MI6Model;
import jason.eis.Point;
import jason.eis.movements.PlannedMovement;
import jason.eis.movements.RandomMovement;
import jason.eis.movements.Search;
import java.util.ArrayList;
import java.util.List;
import java.util.Map;
import java.util.concurrent.*;
import java.util.concurrent.atomic.AtomicReference;
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
    final String agName = ts.getUserAgArch().getAgName();
    final Term outputTerm = terms[terms.length - 1];
    final AtomicReference<Search.TargetType> targetTypeRef = new AtomicReference<>();

    try {
      MI6Model model = MI6Model.getInstance();

      if (DEBUG) logger.info(
        String.format("[%s] Starting RequestGuidance execution", agName)
      );

      // Validate input parameters
      if (terms == null || terms.length < 2) {
        throw new IllegalArgumentException("Invalid number of parameters");
      }

      // Parse parameters with validation
      if (!(terms[0] instanceof Atom)) {
        throw new IllegalArgumentException("First parameter must be an Atom");
      }
      final String targetTypeStr = ((Atom) terms[0]).getFunctor().toUpperCase();

      if (!(terms[1] instanceof NumberTerm)) {
        throw new IllegalArgumentException("Second parameter must be a Number");
      }
      final int size = (int) ((NumberTerm) terms[1]).solve();

      // Get or create path state with null check
      PathState pathState = agentPaths.computeIfAbsent(
        agName,
        k -> new PathState()
      );

      // Get current position and map with null checks
      final LocalMap agentMap = model.getAgentMap(agName);
      if (agentMap == null) {
        logger.warning(String.format("[%s] Agent map is null", agName));
        return handleRandomMovement(agName, un, outputTerm);
      }

      final Point currentPos = agentMap.getCurrentPosition();
      if (currentPos == null) {
        logger.warning(String.format("[%s] Current position is null", agName));
        return handleRandomMovement(agName, un, outputTerm);
      }

      final PlannedMovement plannedMovement = model.getPlannedMovement();
      if (plannedMovement == null) {
        logger.warning(String.format("[%s] Planned movement is null", agName));
        return handleRandomMovement(agName, un, outputTerm);
      }

      // Check cooldown period
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
        return handleRandomMovement(agName, un, outputTerm);
      }

      try {
        CompletableFuture<Search.PathResult> pathFuture = CompletableFuture.supplyAsync(
          () -> {
            try {
              // Convert target type safely
              Search.TargetType targetType = convertTargetType(
                targetTypeStr,
                agName
              );
              targetTypeRef.set(targetType);

              // Find nearest target
              Point target = plannedMovement.findNearestTarget(
                agentMap,
                currentPos,
                targetType
              );

              if (target == null) return null;

              // Calculate path to target
              return plannedMovement.calculatePath(
                agentMap,
                currentPos,
                target,
                targetType
              );
            } catch (Exception e) {
              logger.warning(
                String.format(
                  "[%s] Error in path calculation: %s",
                  agName,
                  e.getMessage()
                )
              );
              return null;
            }
          }
        );

        Search.PathResult pathResult = pathFuture.get(
          PATH_TIMEOUT,
          TimeUnit.MILLISECONDS
        );

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
          pathState.targetType =
            targetTypeRef.get() != null
              ? targetTypeRef.get().toString()
              : "UNKNOWN";

          String nextDirection = pathResult.directions.get(0);
          pathState.currentPath.remove(0);
          return returnSingleDirection(nextDirection, un, outputTerm);
        }
      } catch (TimeoutException e) {
        logger.warning(
          String.format("[%s] Path calculation timed out", agName)
        );
      } catch (Exception e) {
        logger.warning(
          String.format(
            "[%s] Error in path calculation: %s",
            agName,
            e.getMessage()
          )
        );
      }

      // Path calculation failed
      pathState.recordFailedAttempt();
      if (DEBUG) logger.info(
        String.format("[%s] Falling back to random movement", agName)
      );
      return handleRandomMovement(agName, un, outputTerm);
    } catch (Exception e) {
      logger.severe(
        String.format(
          "[%s] Unhandled error in RequestGuidance: %s",
          agName,
          e.getMessage()
        )
      );
      // Always ensure we return a valid movement
      return handleRandomMovement(agName, un, outputTerm);
    }
  }

  private Object handleRandomMovement(
    String agName,
    Unifier un,
    Term outputTerm
  ) {
    try {
      MI6Model model = MI6Model.getInstance();
      RandomMovement randomMovement = model.getRandomMovement();
      LocalMap map = model.getAgentMap(agName);
      String randomDir = randomMovement.getNextDirection(agName, map);
      if (randomDir == null) randomDir = "n"; // Default direction if all else fails

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
    } catch (Exception e) {
      logger.severe(
        String.format(
          "[%s] Error in random movement fallback: %s",
          agName,
          e.getMessage()
        )
      );
      // Ultimate fallback - return north
      return returnSingleDirection("n", un, outputTerm);
    }
  }

  private Object returnSingleDirection(
    String direction,
    Unifier un,
    Term outputTerm
  ) {
    try {
      if (DEBUG) {
        logger.info(String.format("Returning single direction: %s", direction));
      }
      return un.unifies(new Atom(direction), outputTerm);
    } catch (Exception e) {
      logger.severe("Error in direction unification: " + e.getMessage());
      // If unification fails, return false
      return false;
    }
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
