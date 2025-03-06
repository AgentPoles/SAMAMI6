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

  private Term[] terms; // Add this field at class level

  @Override
  public Object execute(TransitionSystem ts, Unifier un, Term[] terms)
    throws Exception {
    final String agName = ts.getUserAgArch().getAgName();
    final AtomicReference<Search.TargetType> targetTypeRef = new AtomicReference<>();

    // Store terms as class field for use in handleRandomMovement
    this.terms = terms;

    try {
      MI6Model model = MI6Model.getInstance();

      if (DEBUG) logger.info(
        String.format("[%s] Starting RequestGuidance execution", agName)
      );

      // Validate input parameters (target, size, blockDir, outputDir)
      if (terms == null || terms.length != 4) {
        throw new IllegalArgumentException(
          "Expected 4 parameters: target, size, blockDir, outputDir"
        );
      }

      // Parse parameters with validation
      if (!(terms[0] instanceof Atom)) {
        throw new IllegalArgumentException(
          "First parameter (target) must be an Atom"
        );
      }
      final String targetTypeStr = ((Atom) terms[0]).getFunctor().toUpperCase();

      if (!(terms[1] instanceof NumberTerm)) {
        throw new IllegalArgumentException(
          "Second parameter (size) must be a Number"
        );
      }
      final int size = (int) ((NumberTerm) terms[1]).solve();

      // Make blockDirection final
      final String blockDirection = terms[2] instanceof Atom
        ? (
          ((Atom) terms[2]).getFunctor().equals("null")
            ? null
            : ((Atom) terms[2]).getFunctor()
        )
        : null;

      // Get and validate agent map
      final LocalMap agentMap = model.getAgentMap(agName);
      if (agentMap == null) {
        logger.warning(String.format("[%s] Agent map is null", agName));
        return handleRandomMovement(agName, un, terms[3]);
      }

      // Update agent state in map
      agentMap.updateAgentState(size, blockDirection);

      // Get current position with null check
      final Point currentPos = agentMap.getCurrentPosition();
      if (currentPos == null) {
        logger.warning(String.format("[%s] Current position is null", agName));
        return handleRandomMovement(agName, un, terms[3]);
      }

      final PlannedMovement plannedMovement = model.getPlannedMovement();
      if (plannedMovement == null) {
        logger.warning(String.format("[%s] Planned movement is null", agName));
        return handleRandomMovement(agName, un, terms[3]);
      }

      // Get or create path state with null check
      PathState pathState = agentPaths.computeIfAbsent(
        agName,
        k -> new PathState()
      );

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
        return handleRandomMovement(agName, un, terms[3]);
      }

      try {
        CompletableFuture<Search.PathResult> pathFuture = CompletableFuture.supplyAsync(
          () -> {
            try {
              Search.TargetType targetType = convertTargetType(
                targetTypeStr,
                agName
              );
              Point target = plannedMovement.findNearestTarget(
                agentMap,
                agentMap.getCurrentPosition(),
                targetType,
                size,
                blockDirection
              );

              if (target == null) return null;

              return plannedMovement.calculatePath(
                agentMap,
                agentMap.getCurrentPosition(),
                target,
                targetType,
                size,
                blockDirection
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
          return returnSingleDirection(nextDirection, un, terms[3]);
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
      return handleRandomMovement(agName, un, terms[3]);
    } catch (Exception e) {
      logger.severe(
        String.format(
          "[%s] Unhandled error in RequestGuidance: %s",
          agName,
          e.getMessage()
        )
      );
      // Always ensure we return a valid movement
      return handleRandomMovement(agName, un, terms[3]);
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

      int size = (int) ((NumberTerm) terms[1]).solve();

      // Get block direction from terms[2]
      String blockDirection = null;
      if (terms[2] instanceof Atom) {
        String dir = ((Atom) terms[2]).getFunctor();
        if (!dir.equals("null")) {
          blockDirection = dir;
        }
      }

      String randomDir = randomMovement.getNextDirection(
        agName,
        map,
        size,
        blockDirection
      );
      if (randomDir == null) randomDir = "n";

      if (DEBUG) {
        logger.info(
          String.format(
            "[%s] Using random movement: direction=%s, size=%d, blockDir=%s",
            agName,
            randomDir,
            size,
            blockDirection
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
