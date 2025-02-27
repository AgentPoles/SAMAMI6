package helpermodels;

import jason.asSemantics.DefaultInternalAction;
import jason.asSemantics.TransitionSystem;
import jason.asSemantics.Unifier;
import jason.asSyntax.*;
import jason.eis.MI6Model;
import jason.eis.Point;
import jason.eis.movements.RandomMovement;
import java.util.ArrayList;
import java.util.List;

public class RequestGuidance extends DefaultInternalAction {
  private static final int AGENT_ONLY = 1;
  private static final int AGENT_WITH_BLOCK = 2;
  private static final int AGENT_WITH_TWO_BLOCKS = 3;
  private static final int AGENT_WITH_BLOCKS_AND_AGENT = 4;

  @Override
  public Object execute(TransitionSystem ts, Unifier un, Term[] terms)
    throws Exception {
    try {
      String agName = ts.getUserAgArch().getAgName();
      MI6Model model = MI6Model.getInstance();

      // Parse target type as atom
      Atom targetType = (Atom) terms[0];
      int size = (int) ((NumberTerm) terms[1]).solve();

      // Optional attachment direction for size > 1
      String attachmentDirection = null;
      if (terms.length > 3 && size > 1) {
        attachmentDirection = ((Atom) terms[2]).getFunctor();
      }

      // Get random direction considering size constraints
      String direction = getRandomDirection(
        agName,
        model,
        size,
        attachmentDirection
      );

      // Unify the result with the output variable
      if (direction != null) {
        return un.unifies(new Atom(direction), terms[terms.length - 1]);
      }

      return false;
    } catch (Exception e) {
      e.printStackTrace();
      return false;
    }
  }

  private String getRandomDirection(
    String agName,
    MI6Model model,
    int size,
    String attachmentDirection
  ) {
    RandomMovement randomMovement = model.getRandomMovement();
    return randomMovement.getNextDirection(agName);
  }

  // Placeholder for future planned movement implementation
  private List<String> getPlannedPath(
    String agName,
    MI6Model model,
    Atom targetType,
    Point targetPos,
    int size,
    String attachmentDirection
  ) {
    // To be implemented later
    return new ArrayList<>();
  }
}
