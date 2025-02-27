package helpermodels;

import jason.asSemantics.DefaultInternalAction;
import jason.asSemantics.TransitionSystem;
import jason.asSemantics.Unifier;
import jason.asSyntax.NumberTerm;
import jason.asSyntax.Term;
import jason.eis.MI6Model;
import jason.eis.Point;

public class AddGoal extends DefaultInternalAction {

  @Override
  public Object execute(TransitionSystem ts, Unifier un, Term[] terms)
    throws Exception {
    try {
      int relX = (int) ((NumberTerm) terms[0]).solve();
      int relY = (int) ((NumberTerm) terms[1]).solve();

      String agName = ts.getUserAgArch().getAgName();

      MI6Model model = MI6Model.getInstance();
      model
        .getAgentMap(agName)
        .addGoal(
          new Point(relX, relY),
          model.getAgentMap(agName).getCurrentPosition()
        );

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
