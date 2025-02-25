package jason.eis;

import eis.AgentListener;
import eis.EnvironmentInterfaceStandard;
import eis.EnvironmentListener;
import eis.exceptions.*;
import eis.iilang.*;
import jason.JasonException;
import jason.NoValueException;
import jason.asSyntax.*;
import jason.environment.Environment;
import java.util.*;
import java.util.logging.Level;
import java.util.logging.Logger;
import massim.eismassim.EnvironmentInterface;

/**
 * This class functions as a Jason environment, using EISMASSim to connect to a MASSim server.
 * (see http://cig.in.tu-clausthal.de/eis)
 * (see also https://multiagentcontest.org)
 *
 * @author Jomi
 * - adapted by ta10
 */
public class EISAdapter extends Environment implements AgentListener {
  private Logger logger = Logger.getLogger(
    "EISAdapter." + EISAdapter.class.getName()
  );

  private EnvironmentInterfaceStandard ei;
  private MI6Model model;

  public EISAdapter() {
    super(20);
  }

  @Override
  public void init(String[] args) {
    ei = new EnvironmentInterface("conf/eismassimconfig.json");
    try {
      ei.start();
      // Initialize model after ei is created
      model = new MI6Model(ei);
    } catch (ManagementException e) {
      e.printStackTrace();
    }

    // Add environment listener
    ei.attachEnvironmentListener(
      new EnvironmentListener() {

        public void handleNewEntity(String entity) {}

        public void handleStateChange(EnvironmentState s) {
          logger.info("new state " + s);
        }

        public void handleDeletedEntity(String arg0, Collection<String> arg1) {}

        public void handleFreeEntity(String arg0, Collection<String> arg1) {}
      }
    );

    // Register agents
    for (String e : ei.getEntities()) {
      System.out.println("Register agent " + e);

      try {
        ei.registerAgent(e);
        ei.attachAgentListener(e, this);
        ei.associateEntity(e, e);
      } catch (Exception e1) {
        e1.printStackTrace();
      }
    }
  }

  @Override
  public void handlePercept(String agent, Percept percept) {}

  @Override
  public List<Literal> getPercepts(String agName) {
    Collection<Literal> ps = super.getPercepts(agName);
    List<Literal> percepts = ps == null
      ? new ArrayList<>()
      : new ArrayList<>(ps);

    clearPercepts(agName);

    if (ei != null) {
      try {
        Map<String, Collection<Percept>> perMap = ei.getAllPercepts(agName);
        for (String entity : perMap.keySet()) {
          Structure strcEnt = ASSyntax.createStructure(
            "entity",
            ASSyntax.createAtom(entity)
          );

          // Update the model's local map with the new percepts
          model.updateLocalMapWithPercepts(agName, perMap.get(entity));

          for (Percept p : perMap.get(entity)) {
            try {
              percepts.add(perceptToLiteral(p).addAnnots(strcEnt));
            } catch (Exception e) {
              logger.warning("Error converting percept: " + e.getMessage());
            }
          }
        }
      } catch (PerceiveException e) {
        logger.log(Level.WARNING, "Could not perceive.", e);
      }
    }
    return percepts;
  }

  @Override
  public boolean executeAction(String agName, Structure action) {
    logger.info("executing: " + action + " by " + agName);

    try {
      String actionName = action.getFunctor();

      switch (actionName) {
        case "move":
          return model.moveTowards(agName, action.getTerm(0).toString());
        case "request":
          return model.requestBlock(agName, action.getTerm(0).toString());
        case "attach":
          return model.attachBlock(agName, action.getTerm(0).toString());
        case "move_best_random_direction":
          int targetX = (int) ((NumberTerm) action.getTerm(0)).solve();
          int targetY = (int) ((NumberTerm) action.getTerm(1)).solve();
          String bestDir = model.chooseBestDirection(agName, targetX, targetY);
          if (bestDir == null) {
            logger.info(
              "Agent " +
              agName +
              " is already moving to a dispenser, skipping random movement."
            );
            return false;
          }
          return model.moveTowards(agName, bestDir);
        case "move_to_nearest_dispenser":
          boolean moving = model.moveToNearestDispenser(agName);
          if (moving) {
            addPercept(
              agName,
              Literal.parseLiteral("moving_to_dispenser(true)")
            );
          } else {
            addPercept(
              agName,
              Literal.parseLiteral("moving_to_dispenser(false)")
            );
          }
          return moving;
        case "find_nearest_dispenser":
          boolean found = model.findNearestDispenser(agName);
          if (found) {
            addPercept(agName, Literal.parseLiteral("found_dispenser(true)"));
          } else {
            addPercept(agName, Literal.parseLiteral("found_dispenser(false)"));
          }
          return found;
        default:
          return super.executeAction(agName, action);
      }
    } catch (Exception e) {
      logger.log(Level.SEVERE, "Error executing action " + action, e);
      return false;
    }
  }

  /** Called before the end of MAS execution */
  @Override
  public void stop() {
    if (ei != null) {
      try {
        if (ei.isKillSupported()) ei.kill();
      } catch (Exception e) {
        e.printStackTrace();
      }
    }
    super.stop();
  }

  private Literal perceptToLiteral(Percept per) throws Exception {
    Literal l = ASSyntax.createLiteral(per.getName());
    for (Parameter param : per.getParameters()) {
      if (param instanceof Numeral) {
        l.addTerm(
          ASSyntax.createNumber(((Numeral) param).getValue().doubleValue())
        );
      } else if (param instanceof Identifier) {
        l.addTerm(ASSyntax.createAtom(((Identifier) param).getValue()));
      }
    }
    return l;
  }

  private static Term parameterToTerm(Parameter par) throws JasonException {
    if (par instanceof Numeral) {
      return ASSyntax.createNumber(((Numeral) par).getValue().doubleValue());
    } else if (par instanceof Identifier) {
      try {
        Identifier i = (Identifier) par;
        String a = i.getValue();
        if (!Character.isUpperCase(a.charAt(0))) return ASSyntax.parseTerm(a);
      } catch (Exception ignored) {}
      return ASSyntax.createString(((Identifier) par).getValue());
    } else if (par instanceof ParameterList) {
      ListTerm list = new ListTermImpl();
      ListTerm tail = list;
      for (Parameter p : (ParameterList) par) tail =
        tail.append(parameterToTerm(p));
      return list;
    } else if (par instanceof Function) {
      Function f = (Function) par;
      Structure l = ASSyntax.createStructure(f.getName());
      for (Parameter p : f.getParameters()) l.addTerm(parameterToTerm(p));
      return l;
    }
    throw new JasonException("The type of parameter " + par + " is unknown!");
  }

  private static Action literalToAction(Literal action) {
    Parameter[] pars = new Parameter[action.getArity()];
    for (int i = 0; i < action.getArity(); i++) pars[i] =
      termToParameter(action.getTerm(i));
    return new Action(action.getFunctor(), pars);
  }

  private static Parameter termToParameter(Term t) {
    if (t.isNumeric()) {
      try {
        double d = ((NumberTerm) t).solve();
        if ((d == Math.floor(d)) && !Double.isInfinite(d)) return new Numeral(
          (int) d
        );
        return new Numeral(d);
      } catch (NoValueException e) {
        e.printStackTrace();
      }
      return new Numeral(null);
    } else if (t.isList()) {
      Collection<Parameter> terms = new ArrayList<>();
      for (Term listTerm : (ListTerm) t) terms.add(termToParameter(listTerm));
      return new ParameterList(terms);
    } else if (t.isString()) {
      return new Identifier(((StringTerm) t).getString());
    } else if (t.isLiteral()) {
      Literal l = (Literal) t;
      if (!l.hasTerm()) {
        return new Identifier(l.getFunctor());
      } else {
        Parameter[] terms = new Parameter[l.getArity()];
        for (int i = 0; i < l.getArity(); i++) terms[i] =
          termToParameter(l.getTerm(i));
        return new Function(l.getFunctor(), terms);
      }
    }
    return new Identifier(t.toString());
  }
}
