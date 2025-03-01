package helpermodels;

import jason.asSemantics.DefaultInternalAction;
import jason.asSemantics.TransitionSystem;
import jason.asSemantics.Unifier;
import jason.asSyntax.Term;
import jason.eis.LocalMap;
import jason.eis.MI6Model;
import jason.eis.movements.AgentCollisionHandler;
import java.util.logging.Logger;

public class AddMovementFailure extends DefaultInternalAction {
  private static final Logger logger = Logger.getLogger(
    AddMovementFailure.class.getName()
  );
  private static MI6Model model = MI6Model.getInstance();
  private static AgentCollisionHandler collisionHandler = new AgentCollisionHandler();

  @Override
  public Object execute(TransitionSystem ts, Unifier un, Term[] args)
    throws Exception {
    try {
      String agName = ts.getUserAgArch().getAgName();
      String failureType = args[0].toString();
      String attemptedDirection = args[1].toString();

      LocalMap map = model.getAgentMap(agName);
      if (map == null) {
        logger.warning("No map found for agent: " + agName);
        return false;
      }

      collisionHandler.recordFailedMove(
        agName,
        map.getCurrentPosition(),
        attemptedDirection
      );

      switch (failureType) {
        case "failed_forbidden":
          logger.info("Agent " + agName + " hit boundary");
          map.handleBoundaryFailure(attemptedDirection);
          break;
        case "failed_path":
          logger.info("Agent " + agName + " path blocked");
          map.handlePathFailure();
          break;
        default:
          logger.warning("Unknown failure type: " + failureType);
          break;
      }
      return true;
    } catch (Exception e) {
      logger.severe("Error handling movement failure: " + e.getMessage());
      return false;
    }
  }
}
