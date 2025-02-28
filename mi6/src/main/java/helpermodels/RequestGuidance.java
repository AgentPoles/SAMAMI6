package helpermodels;

import jason.asSemantics.DefaultInternalAction;
import jason.asSemantics.TransitionSystem;
import jason.asSemantics.Unifier;
import jason.asSyntax.*;
import jason.eis.MI6Model;
import jason.eis.Point;
import jason.eis.movements.RandomMovement;
import jason.eis.movements.Search;
import java.util.*;
import java.util.concurrent.*;
import java.util.logging.Level;
import java.util.logging.Logger;

public class RequestGuidance extends DefaultInternalAction {
  private static final Logger logger = Logger.getLogger(
    RequestGuidance.class.getName()
  );

  private static final int AGENT_ONLY = 1;
  private static final int AGENT_WITH_BLOCK = 2;
  private static final int AGENT_WITH_TWO_BLOCKS = 3;
  private static final int AGENT_WITH_BLOCKS_AND_AGENT = 4;

  // Path calculation management
  private final ExecutorService pathfindingExecutor = Executors.newFixedThreadPool(
    2
  );
  private final Map<String, Future<List<String>>> pendingCalculations = new ConcurrentHashMap<>();
  private final Map<String, Long> lastRequestTime = new ConcurrentHashMap<>();
  private static final long REQUEST_COOLDOWN = 1000; // milliseconds
  private static final long CALCULATION_TIMEOUT = 2000; // 2 seconds timeout

  @Override
  public Object execute(TransitionSystem ts, Unifier un, Term[] terms)
    throws Exception {
    try {
      String agName = ts.getUserAgArch().getAgName();
      MI6Model model = MI6Model.getInstance();
      RandomMovement randomMovement = model.getRandomMovement(); // Get shared RandomMovement instance

      // Parse parameters
      Atom targetTypeAtom = (Atom) terms[0];
      int size = (int) ((NumberTerm) terms[1]).solve();

      // Convert target type to search type
      Search.TargetType targetType = convertTargetType(
        targetTypeAtom.getFunctor()
      );

      logger.info(
        agName +
        ": Requesting guidance for " +
        targetType +
        " with size " +
        size
      );

      long now = System.currentTimeMillis();

      // Use RandomMovement during cooldown
      if (lastRequestTime.containsKey(agName)) {
        long timeSinceLastRequest = now - lastRequestTime.get(agName);
        if (timeSinceLastRequest < REQUEST_COOLDOWN) {
          String randomDir = randomMovement.getNextDirection(agName);
          logger.info(
            agName +
            ": Using RandomMovement (cooldown active), direction: " +
            randomDir
          );
          return Collections.singletonList(randomDir != null ? randomDir : "n");
        }
      }

      // Check pending calculations
      Future<List<String>> pendingCalc = pendingCalculations.get(agName);
      if (pendingCalc != null && !pendingCalc.isDone()) {
        try {
          List<String> result = pendingCalc.get(100, TimeUnit.MILLISECONDS);
          if (!result.isEmpty()) {
            pendingCalculations.remove(agName);
            lastRequestTime.put(agName, now);
            return result;
          }
        } catch (TimeoutException te) {
          logger.warning(agName + ": Timeout getting calculation result");
        } catch (Exception e) {
          logger.log(Level.WARNING, agName + ": Path calculation failed", e);
        }
      }

      // Start new path calculation
      Future<List<String>> calculation = pathfindingExecutor.submit(
        () -> model.requestGuidance(agName, targetType.ordinal(), size)
      );

      pendingCalculations.put(agName, calculation);
      lastRequestTime.put(agName, now);

      // Use RandomMovement while waiting for path calculation
      String randomDir = randomMovement.getNextDirection(agName);
      logger.info(
        agName +
        ": Using RandomMovement while calculating, direction: " +
        randomDir
      );
      return Collections.singletonList(randomDir != null ? randomDir : "n");
    } catch (Exception e) {
      logger.log(Level.SEVERE, "Error in RequestGuidance execution", e);
      throw e;
    }
  }

  private Search.TargetType convertTargetType(String type) {
    switch (type.toLowerCase()) {
      case "dispenser":
        return Search.TargetType.DISPENSER;
      case "block":
        return Search.TargetType.BLOCK;
      case "goal":
        return Search.TargetType.GOAL;
      default:
        logger.warning(
          "Unknown target type: " + type + ", defaulting to DISPENSER"
        );
        return Search.TargetType.DISPENSER;
    }
  }

  @Override
  public void destroy() {
    logger.info("Shutting down RequestGuidance pathfinding executor");
    pathfindingExecutor.shutdown();
    try {
      if (!pathfindingExecutor.awaitTermination(800, TimeUnit.MILLISECONDS)) {
        logger.warning("Forcing pathfinding executor shutdown");
        pathfindingExecutor.shutdownNow();
      }
    } catch (InterruptedException e) {
      logger.warning("Interrupted while shutting down pathfinding executor");
      pathfindingExecutor.shutdownNow();
    }
  }
}
