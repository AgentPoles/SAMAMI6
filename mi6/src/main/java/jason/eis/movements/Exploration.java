package jason.eis.movements;

import jason.eis.LocalMap;
import jason.eis.Point;
import java.util.*;
import java.util.concurrent.ConcurrentHashMap;

public class Exploration {
    private static final Map<String, ExplorationState> agentStates = new ConcurrentHashMap<>();
    private static final int VIEW_RANGE = 5;
    private static final int BOUNDARY_DETECTION_RANGE = 3;
    private static final double BOUNDARY_REPULSION = 3.0;
    private static final int DIRECTION_CHANGE_THRESHOLD = 5;
    private static final double LEVY_FLIGHT_PROBABILITY = 0.2;
    private static final double EXPLORATION_DECAY_RATE = 0.95;
    private static final int INTEREST_ZONE_RADIUS = 8;
    
    private static class ExplorationState {
        Set<Point> exploredCells;
        Map<Point, Double> explorationHeat;
        Point lastPosition;
        String lastDirection;
        int stepsInDirection;
        int stuckCount;
        List<String> directionHistory;
        Set<Point> visitedPositions;
        int longJumpCooldown;
        Set<Point> interestZones;
        
        ExplorationState() {
            this.exploredCells = new HashSet<>();
            this.explorationHeat = new HashMap<>();
            this.stuckCount = 0;
            this.stepsInDirection = 0;
            this.directionHistory = new ArrayList<>();
            this.visitedPositions = new HashSet<>();
            this.longJumpCooldown = 0;
            this.interestZones = new HashSet<>();
        }
        
        void updateHeatMap(Point pos) {
            explorationHeat.merge(pos, 1.0, Double::sum);
            // Decay heat values
            explorationHeat.replaceAll((k, v) -> v * EXPLORATION_DECAY_RATE);
            // Remove negligible heat values
            explorationHeat.entrySet().removeIf(e -> e.getValue() < 0.1);
        }
        
        double getHeatValue(Point pos) {
            return explorationHeat.getOrDefault(pos, 0.0);
        }
        
        void addInterestZone(Point pos) {
            interestZones.add(pos);
        }
        
        void addDirection(String direction) {
            directionHistory.add(direction);
            if (directionHistory.size() > 10) {
                directionHistory.remove(0);
            }
        }
        
        boolean isOscillating() {
            if (directionHistory.size() < 4) return false;
            String last = directionHistory.get(directionHistory.size() - 1);
            String secondLast = directionHistory.get(directionHistory.size() - 2);
            return last.equals(getOppositeDirection(secondLast));
        }
    }

    public static String getRecommendedDirection(String agName, LocalMap map, Point currentPos) {
        if (map == null || currentPos == null) return null;
        
        ExplorationState state = agentStates.computeIfAbsent(agName, k -> new ExplorationState());
        
        // Update state
        updateState(state, map, currentPos);
        
        // Get available directions
        List<String> availableDirections = getAvailableDirections(map, currentPos);
        if (availableDirections.isEmpty()) return null;

        // Check for Lévy flight opportunity
        boolean isLevyFlight = shouldTakeLevyFlight(state);
        
        // Calculate scores for each direction
        Map<String, Double> scores = new HashMap<>();
        for (String direction : availableDirections) {
            if (direction != null) {
                Point targetPos = getTargetPosition(currentPos, direction);
                double score = calculateDirectionScore(state, map, currentPos, targetPos, direction, isLevyFlight);
                scores.put(direction, score);
            }
        }

        String chosenDirection = chooseBestDirection(scores, state);
        if (chosenDirection != null) {
            updateStateWithNewDirection(state, chosenDirection, currentPos);
        }
        
        return chosenDirection;
    }

    private static void updateState(ExplorationState state, LocalMap map, Point currentPos) {
        // Update stuck detection
        if (state.lastPosition != null && state.lastPosition.equals(currentPos)) {
            state.stuckCount++;
        } else {
            state.stuckCount = 0;
        }
        
        // Update visited positions
        state.visitedPositions.add(currentPos);
        
        // Update steps in current direction
        if (state.lastDirection != null) {
            state.stepsInDirection++;
        }
        
        // Update explored cells
        updateExploredArea(state, map, currentPos);
    }

    private static boolean shouldTakeLevyFlight(ExplorationState state) {
        if (state.longJumpCooldown > 0) {
            state.longJumpCooldown--;
            return false;
        }
        
        boolean takeLevy = Math.random() < LEVY_FLIGHT_PROBABILITY;
        if (takeLevy) {
            state.longJumpCooldown = 5; // Set cooldown period
        }
        return takeLevy;
    }

    private static double calculateDirectionScore(ExplorationState state, LocalMap map, 
            Point currentPos, Point targetPos, String direction, boolean isLevyFlight) {
        if (direction == null) return Double.NEGATIVE_INFINITY;
        
        double score = 0.0;

        // Base exploration score based on heat map
        double heatValue = state.getHeatValue(targetPos);
        score += (1.0 - Math.min(1.0, heatValue)) * 2.0;

        // Lévy flight bonus for longer jumps
        if (isLevyFlight) {
            score += calculateLevyFlightScore(state, map, currentPos, direction) * 3.0;
        }

        // Interest zone influence
        score += calculateInterestZoneScore(state, targetPos) * 1.5;

        // Boundary handling with bounce patterns
        if (isNearBoundary(map, targetPos)) {
            score += calculateBounceScore(state, map, currentPos, direction);
        }

        // Open area preference with dynamic weighting
        double openAreaScore = scoreOpenArea(map, targetPos);
        score += openAreaScore * (isLevyFlight ? 2.0 : 1.0);

        // Momentum and pattern breaking
        score += calculateMomentumScore(state, direction);

        // Add small random factor
        score += Math.random() * 0.3;

        return score;
    }

    private static double calculateLevyFlightScore(ExplorationState state, LocalMap map, Point currentPos, String direction) {
        int distance = 0;
        Point pos = currentPos;
        
        // Look ahead in the current direction
        for (int i = 0; i < 5; i++) {
            Point nextPos = getTargetPosition(pos, direction);
            if (map.hasObstacle(nextPos) || map.isOutOfBounds(nextPos)) break;
            
            distance++;
            if (!state.visitedPositions.contains(nextPos)) {
                return distance * 0.5;
            }
            pos = nextPos;
        }
        
        return 0.0;
    }

    private static double calculateInterestZoneScore(ExplorationState state, Point targetPos) {
        double score = 0.0;
        for (Point zone : state.interestZones) {
            double distance = euclideanDistance(targetPos, zone);
            if (distance < INTEREST_ZONE_RADIUS) {
                score += (INTEREST_ZONE_RADIUS - distance) / INTEREST_ZONE_RADIUS;
            }
        }
        return score;
    }

    private static double calculateBounceScore(ExplorationState state, LocalMap map, Point currentPos, String direction) {
        // If moving towards boundary, prefer perpendicular directions
        if (isMovingTowardsBoundary(map, currentPos, direction)) {
            String[] perpendicular = getPerpendicularDirections(direction);
            for (String dir : perpendicular) {
                Point checkPos = getTargetPosition(currentPos, dir);
                if (!map.hasObstacle(checkPos) && !map.isOutOfBounds(checkPos)) {
                    return 1.0;
                }
            }
            return -BOUNDARY_REPULSION;
        }
        return 0.0;
    }

    private static String[] getPerpendicularDirections(String direction) {
        return switch (direction) {
            case "n", "s" -> new String[]{"e", "w"};
            case "e", "w" -> new String[]{"n", "s"};
            default -> new String[]{};
        };
    }

    private static boolean isMovingTowardsBoundary(LocalMap map, Point pos, String direction) {
        Point nextPos = getTargetPosition(pos, direction);
        return isNearBoundary(map, nextPos) && !isNearBoundary(map, pos);
    }

    private static double calculateMomentumScore(ExplorationState state, String direction) {
        double score = 0.0;
        
        // Basic momentum
        if (direction.equals(state.lastDirection) && state.stepsInDirection < DIRECTION_CHANGE_THRESHOLD) {
            score += 0.5;
        }
        
        // Pattern breaking
        if (state.isOscillating() && direction.equals(state.lastDirection)) {
            score -= 1.0;
        }
        
        return score;
    }

    private static double calculateDirectionScore(ExplorationState state, LocalMap map, 
            Point currentPos, Point targetPos, String direction) {
        if (direction == null) return Double.NEGATIVE_INFINITY;
        
        double score = 0.0;

        // Strong boundary avoidance
        if (isNearBoundary(map, targetPos)) {
            score -= BOUNDARY_REPULSION;
        }

        // Exploration score - prefer unvisited positions
        if (!state.visitedPositions.contains(targetPos)) {
            score += 2.0;
        }

        // Momentum score - but not too much
        if (direction.equals(state.lastDirection) && state.stepsInDirection < DIRECTION_CHANGE_THRESHOLD) {
            score += 0.5;
        }

        // Penalize backtracking unless necessary
        if (state.lastDirection != null) {
            String oppositeDir = getOppositeDirection(state.lastDirection);
            if (direction.equals(oppositeDir)) {
                score -= 1.0;
            }
        }

        // Prefer directions that lead to open areas
        score += scoreOpenArea(map, targetPos);

        // Add randomization to break patterns
        score += Math.random() * 0.5;

        return score;
    }

    private static double scoreOpenArea(LocalMap map, Point pos) {
        int openSpaces = 0;
        for (int dx = -1; dx <= 1; dx++) {
            for (int dy = -1; dy <= 1; dy++) {
                if (dx == 0 && dy == 0) continue;
                Point checkPos = new Point(pos.x + dx, pos.y + dy);
                if (!map.hasObstacle(checkPos) && !map.isOutOfBounds(checkPos)) {
                    openSpaces++;
                }
            }
        }
        return openSpaces * 0.3;
    }

    private static boolean isNearBoundary(LocalMap map, Point pos) {
        for (int dx = -BOUNDARY_DETECTION_RANGE; dx <= BOUNDARY_DETECTION_RANGE; dx++) {
            for (int dy = -BOUNDARY_DETECTION_RANGE; dy <= BOUNDARY_DETECTION_RANGE; dy++) {
                Point checkPos = new Point(pos.x + dx, pos.y + dy);
                if (map.isOutOfBounds(checkPos)) {
                    return true;
                }
            }
        }
        return false;
    }

    private static void updateStateWithNewDirection(ExplorationState state, String direction, Point currentPos) {
        if (direction == null) return;
        
        if (!direction.equals(state.lastDirection)) {
            state.stepsInDirection = 0;
        }
        state.lastDirection = direction;
        state.lastPosition = currentPos;
        state.addDirection(direction);
    }

    private static void updateExploredArea(ExplorationState state, LocalMap map, Point currentPos) {
        // Add visible area to explored cells
        for (int dx = -VIEW_RANGE; dx <= VIEW_RANGE; dx++) {
            for (int dy = -VIEW_RANGE; dy <= VIEW_RANGE; dy++) {
                Point checkPoint = new Point(currentPos.x + dx, currentPos.y + dy);
                if (!map.isOutOfBounds(checkPoint)) {
                    state.exploredCells.add(checkPoint);
                }
            }
        }
    }

    private static String chooseBestDirection(Map<String, Double> scores, ExplorationState state) {
        return scores.entrySet()
            .stream()
            .max(Map.Entry.comparingByValue())
            .map(Map.Entry::getKey)
            .orElse(null);
    }

    private static List<String> getAvailableDirections(LocalMap map, Point currentPos) {
        List<String> directions = new ArrayList<>();
        String[] possibleDirs = {"n", "s", "e", "w"};
        
        for (String dir : possibleDirs) {
            Point targetPos = getTargetPosition(currentPos, dir);
            if (!map.hasObstacle(targetPos) && !map.isOutOfBounds(targetPos)) {
                directions.add(dir);
            }
        }
        return directions;
    }

    private static Point getTargetPosition(Point current, String direction) {
        return switch (direction) {
            case "n" -> new Point(current.x, current.y - 1);
            case "s" -> new Point(current.x, current.y + 1);
            case "e" -> new Point(current.x + 1, current.y);
            case "w" -> new Point(current.x - 1, current.y);
            default -> current;
        };
    }

    private static String getOppositeDirection(String direction) {
        if (direction == null) return null;
        
        return switch (direction) {
            case "n" -> "s";
            case "s" -> "n";
            case "e" -> "w";
            case "w" -> "e";
            default -> null;
        };
    }

    private static double euclideanDistance(Point p1, Point p2) {
        int dx = p2.x - p1.x;
        int dy = p2.y - p1.y;
        return Math.sqrt(dx * dx + dy * dy);
    }
} 