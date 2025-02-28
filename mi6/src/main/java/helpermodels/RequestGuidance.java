package helpermodels;

import jason.asSemantics.DefaultInternalAction;
import jason.asSemantics.TransitionSystem;
import jason.asSemantics.Unifier;
import jason.asSyntax.*;
import jason.eis.MI6Model;
import jason.eis.Point;
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

      // Get random direction
      String direction = getRandomDirection(agName, model);

      // Create a list with single direction for now
      ListTerm dirList = new ListTermImpl();
      dirList.add(new Atom(direction));

      // Unify with the output variable
      return un.unifies(dirList, terms[2]);
    } catch (Exception e) {
      e.printStackTrace();
      return false;
    }
  }

  private String getRandomDirection(String agName, MI6Model model) {
    // For now, just return a random direction
    String[] directions = { "n", "s", "e", "w" };
    int randomIndex = (int) (Math.random() * directions.length);
    return directions[randomIndex];
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
