package helpermodels;

import jason.asSemantics.DefaultInternalAction;
import jason.asSemantics.TransitionSystem;
import jason.asSemantics.Unifier;
import jason.asSyntax.Term;
import jason.eis.MI6Model;

public class UpdateMovement extends DefaultInternalAction {

  @Override
  public Object execute(TransitionSystem ts, Unifier un, Term[] terms)
    throws Exception {
    try {
      String direction = terms[0].toString().toLowerCase();
      String agName = ts.getUserAgArch().getAgName();

      MI6Model model = MI6Model.getInstance();
      model.getAgentMap(agName).updatePositionFromMovement(direction);

      if (MI6Model.DEBUG) {
        model.logMapState(agName);
      }

      return true;
    } catch (Exception e) {
      e.printStackTrace();
      return false;
    }
  }
}
