package jason.eis.movements;

import jason.eis.LocalMap;

public interface MovementStrategy {
  String getNextMove(String agName, LocalMap map);
}
