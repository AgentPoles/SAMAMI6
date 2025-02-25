package jason.eis;

import eis.EnvironmentInterfaceStandard;
import eis.iilang.*;
import jason.asSyntax.*;
import jason.environment.Environment;
import java.util.*;
import java.util.logging.Level;
import java.util.logging.Logger;

public class MI6Model {
  private Logger logger = Logger.getLogger(
    "MI6Model." + MI6Model.class.getName()
  );
  private EnvironmentInterfaceStandard ei;
  private Random random = new Random();
  private static final String[] DIRECTIONS = { "n", "s", "e", "w" };

  public MI6Model(EnvironmentInterfaceStandard ei) {
    this.ei = ei;
  }

  public boolean moveTowards(String agName, String direction) {
    try {
      ei.performAction(agName, new Action("move", new Identifier(direction)));
      return true;
    } catch (Exception e) {
      logger.log(Level.SEVERE, "Error in moveTowards", e);
      return false;
    }
  }

  public boolean requestBlock(String agName, String direction) {
    try {
      ei.performAction(
        agName,
        new Action("request", new Identifier(direction))
      );
      return true;
    } catch (Exception e) {
      logger.log(Level.SEVERE, "Error in requestBlock", e);
      return false;
    }
  }

  public boolean attachBlock(String agName, String direction) {
    try {
      ei.performAction(agName, new Action("attach", new Identifier(direction)));
      return true;
    } catch (Exception e) {
      logger.log(Level.SEVERE, "Error in attachBlock", e);
      return false;
    }
  }

  public String chooseBestDirection(String agName, int targetX, int targetY) {
    try {
      // If we're aligned with target (same X or Y coordinate), move directly towards it
      if (targetX == 0) {
        if (targetY < 0) return "n";
        if (targetY > 0) return "s";
      }
      if (targetY == 0) {
        if (targetX < 0) return "w";
        if (targetX > 0) return "e";
      }

      // If not aligned, check for obstacles and do random movement
      Map<String, Double> risks = evaluateRisks(agName, targetX, targetY);
      List<String> safeDirections = new ArrayList<>();

      for (Map.Entry<String, Double> entry : risks.entrySet()) {
        if (entry.getValue() < 1000.0) { // If not blocked
          safeDirections.add(entry.getKey());
        }
      }

      // If we have safe directions, choose randomly among them
      if (!safeDirections.isEmpty()) {
        return safeDirections.get(random.nextInt(safeDirections.size()));
      }

      // If all directions are blocked, just pick any direction
      return DIRECTIONS[random.nextInt(DIRECTIONS.length)];
    } catch (Exception e) {
      logger.log(Level.SEVERE, "Error in chooseBestDirection", e);
      return DIRECTIONS[random.nextInt(DIRECTIONS.length)];
    }
  }

  private Map<String, Double> evaluateRisks(
    String agName,
    int targetX,
    int targetY
  )
    throws Exception {
    // Initialize risks
    Map<String, Double> risks = new HashMap<>();
    risks.put("n", 1.0);
    risks.put("s", 1.0);
    risks.put("e", 1.0);
    risks.put("w", 1.0);

    // Check for obstacles and other agents
    Map<String, Collection<Percept>> percepts = ei.getAllPercepts(agName);
    for (Collection<Percept> perceptList : percepts.values()) {
      for (Percept p : perceptList) {
        if (p.getName().equals("thing")) {
          Parameter[] params = p.getParameters().toArray(new Parameter[0]);
          int x = ((Numeral) params[0]).getValue().intValue();
          int y = ((Numeral) params[1]).getValue().intValue();
          String type = ((Identifier) params[2]).getValue();

          // Mark directions with obstacles as very high risk
          if (type.equals("obstacle") || type.equals("entity")) {
            if (x == 0 && y == -1) risks.put("n", 1000.0);
            if (x == 0 && y == 1) risks.put("s", 1000.0);
            if (x == 1 && y == 0) risks.put("e", 1000.0);
            if (x == -1 && y == 0) risks.put("w", 1000.0);
          }
        }
      }
    }

    // Adjust risks based on target direction
    if (targetX > 0) risks.put("e", Math.min(risks.get("e"), 0.5));
    if (targetX < 0) risks.put("w", Math.min(risks.get("w"), 0.5));
    if (targetY > 0) risks.put("s", Math.min(risks.get("s"), 0.5));
    if (targetY < 0) risks.put("n", Math.min(risks.get("n"), 0.5));

    return risks;
  }
}
