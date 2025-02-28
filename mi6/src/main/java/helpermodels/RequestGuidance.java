package helpermodels;

import jason.asSemantics.DefaultInternalAction;
import jason.asSemantics.TransitionSystem;
import jason.asSemantics.Unifier;
import jason.asSyntax.*;
import jason.eis.MI6Model;
import jason.eis.movements.RandomMovement;
import java.util.logging.Level;
import java.util.logging.Logger;

public class RequestGuidance extends DefaultInternalAction {
  private static final Logger logger = Logger.getLogger(
    RequestGuidance.class.getName()
  );

  @Override
  public Object execute(TransitionSystem ts, Unifier un, Term[] terms)
    throws Exception {
    try {
      String agName = ts.getUserAgArch().getAgName();
      MI6Model model = MI6Model.getInstance();
      RandomMovement randomMovement = model.getRandomMovement();

      // Parse target type
      String targetType = ((Atom) terms[0]).getFunctor();

      // Parse size
      int size = (int) ((NumberTerm) terms[1]).solve();

      // Get attachment direction if size > 1
      String attachDir = null;
      Term outputTerm;
      if (size > 1 && terms.length > 3) {
        attachDir = ((Atom) terms[2]).getFunctor();
        outputTerm = terms[3];
      } else {
        outputTerm = terms[2];
      }

      // Get random direction
      String randomDir = randomMovement.getNextDirection(agName);
      if (randomDir == null) randomDir = "n";

      logger.info(
        agName +
        ": Using RandomMovement for " +
        targetType +
        ", size=" +
        size +
        (attachDir != null ? ", attached=" + attachDir : "") +
        ", direction=" +
        randomDir
      );

      // Create a list with single direction as an Atom
      ListTerm dirList = new ListTermImpl();
      dirList.add(new Atom(randomDir));

      // Unify with output variable
      return un.unifies(dirList, outputTerm);
    } catch (Exception e) {
      logger.log(Level.SEVERE, "Error in RequestGuidance execution", e);
      throw e;
    }
  }
}
