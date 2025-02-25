//monday - 10:53
// Initial beliefs and rules
current_pos(0,0).
map_initialized(false).
initialization_steps(3).
phase(initial).
current_path([]).  // Initialize empty path

// Add this belief to track failed moves
failed_moves(0).

// Percept handlers with atom types
+thing(X,Y,dispenser,b0)[source(percept)] : true <-
  ?current_pos(CX,CY);
  +local_map(CX+X,CY+Y,db0).


+thing(X,Y,dispenser,b1)[source(percept)] : true <-
  ?current_pos(CX,CY);
  +local_map(CX+X,CY+Y,db1).


+thing(X,Y,entity,"A")[source(percept)] : true <-
  ?current_pos(CX,CY);
  +local_map(CX+X,CY+Y,a).


+obstacle(X,Y)[source(percept)] : true <-
  ?current_pos(CX,CY);
  +local_map(CX+X,CY+Y,ob).


+goal(X,Y)[source(percept)] : true <-
  ?current_pos(CX,CY);
  +local_map(CX+X,CY+Y,goal).


// Initial step handling with phase transition
+step(S) : phase(initial) <-
  ?initialization_steps(WaitSteps);
   if (S >= WaitSteps) {
       -+map_initialized(true);
       -+phase(planning);
      !plan_next_move;
   }.


// Regular step handling - trigger planning when needed
@switching_to_planning[atomic]
+!switch_to_planning_mode : true <-
   -+phase(planning).


// Main planning decision
+!plan_next_move : phase(planning) <-
  .my_name(Name);
  ?current_pos(MyX,MyY);
  .findall(
       [X,Y,Type],
       (local_map(X,Y,Type) & (Type == db0 | Type == db1)),
       Dispensers
   );
   if (not .empty(Dispensers)) {
      .findall([X,Y],
           (local_map(X,Y,ob) | local_map(X,Y,a)),
           Blocked
       );
      .print("Agent ", Name, " at ", MyX, ",", MyY, " planning dispenser move");
      !plan_dispenser_path(Dispensers, Blocked, MyX, MyY);
   } else {
      .print("Agent ", Name, " at ", MyX, ",", MyY, " planning exploration move");
      !plan_exploration_path(MyX, MyY);
   };
   -+phase(moving).


// Helper rule to check if a path is valid - simplified version
+?is_valid_path(Path) : true <-
  .print("Checking path validity for: ", Path);
   if (not .list(Path)) {
      .print("Path is not a list");
       false;
   } elif (Path = []) {
      .print("Path is empty");
       false;
   } else {
      .print("Path is valid list with moves: ", Path);
       true;
   }.


// Dispenser pathfinding with retries
+!plan_dispenser_path(Dispensers, Blocked, MyX, MyY) : true <-
  .my_name(Name);
  !attempt_dispenser_path(Dispensers, Blocked, MyX, MyY, 2).


// Modified helper plan to attempt dispenser pathfinding with retry count
+!attempt_dispenser_path(Dispensers, Blocked, MyX, MyY, Attempts) : true <-
    .my_name(Name);
    // First, filter out dispensers that have agents at their location
    .findall([X,Y,Type],
        (.member([X,Y,Type], Dispensers) & not local_map(X,Y,a)),
        UnoccupiedDispensers
    );
    
    // Then calculate distances for unoccupied dispensers
    .findall([D,X,Y,Type],
        (.member([X,Y,Type], UnoccupiedDispensers) & D = math.abs(X-MyX) + math.abs(Y-MyY)),
        AccessibleDispensers
    );
    
    if (not .empty(AccessibleDispensers)) {
        ?find_minimum_distance(AccessibleDispensers, [_,TargetX,TargetY,Type]);
        !generate_path(MyX, MyY, TargetX, TargetY, Blocked, [], GeneratedPath);
        
        // Rest of the existing logic remains the same
        if (GeneratedPath \== [] & .length(GeneratedPath, L) & L <= 8) {
            -+current_path(GeneratedPath);
            -+has_target_dispenser(TargetX,TargetY,Type);
            .print("Agent ", Name, " generated valid dispenser path: ", GeneratedPath);
        } else {
            // Existing intermediate target logic...
            DX = TargetX - MyX;
            DY = TargetY - MyY;
            IntermediateX = MyX + (DX div 2);
            IntermediateY = MyY + (DY div 2);
            !generate_path(MyX, MyY, IntermediateX, IntermediateY, Blocked, [], ShorterPath);
            
            if (ShorterPath \== [] & .length(ShorterPath, L2) & L2 <= 4) {
                -+current_path(ShorterPath);
                -+has_target_dispenser(TargetX,TargetY,Type);
                .print("Agent ", Name, " generated intermediate dispenser path: ", ShorterPath);
            } else {
                .print("Agent ", Name, " failed to generate valid path to dispenser, attempts left: ", Attempts-1);
                if (Attempts > 1) {
                    !attempt_dispenser_path(Dispensers, Blocked, MyX, MyY, Attempts-1);
                } else {
                    .print("Agent ", Name, " failed all attempts to reach dispenser, falling back to exploration");
                    !plan_exploration_path(MyX, MyY);
                };
            };
        };
    } else {
        .print("Agent ", Name, " found no unoccupied dispensers, falling back to exploration");
        !plan_exploration_path(MyX, MyY);
    }.


// Simple exploration path planning with improved validation
+!plan_exploration_path(MyX, MyY) : true <-
  .my_name(Name);
   // Use smaller random ranges for more manageable paths
  .random([-3,-2,-1,1,2,3], DX);
  .random([-3,-2,-1,1,2,3], DY);
   TargetX = MyX + DX;
   TargetY = MyY + DY;
  .print("Agent ", Name, " planning path to ", TargetX, ",", TargetY);
  
   // Get blocked positions
  .findall([X,Y],
       (local_map(X,Y,ob) | local_map(X,Y,a)),
       Blocked
   );
  
   // Generate and validate path
  !generate_path(MyX, MyY, TargetX, TargetY, Blocked, [], GeneratedPath);
   if (GeneratedPath \== [] & .length(GeneratedPath, L) & L <= 5) {
       // Accept path if it's non-empty and reasonably short
       -+current_path(GeneratedPath);
      .print("Agent ", Name, " generated valid path: ", GeneratedPath);
   } else {
       // Try with closer target
       CloserX = MyX + (DX div 2);
       CloserY = MyY + (DY div 2);
      .print("Retrying with closer target: ", CloserX, ",", CloserY);
      !generate_path(MyX, MyY, CloserX, CloserY, Blocked, [], GeneratedPath2);
      
       if (GeneratedPath2 \== [] & .length(GeneratedPath2, L2) & L2 <= 3) {
           -+current_path(GeneratedPath2);
          .print("Agent ", Name, " generated valid path on retry: ", GeneratedPath2);
       } else {
           // Fall back to single random move
          .random([n,s,e,w], Move);
           -+current_path([Move]);
          .print("Agent ", Name, " falling back to single random move: ", Move);
       };
   }.


// Action handling with safer path checking
+actionID(X) : phase(moving) & current_path([H|T]) <-
  .my_name(Name);
  ?current_pos(CX,CY);
  .print("Agent ", Name, " executing move ", H, " from ", CX, ",", CY, " path remaining: ", T);
  !move(H);
  !update_path(T).


+actionID(X) : phase(planning) <-
  .my_name(Name);
  ?phase(P);
  ?current_path(Path);
  .print("Agent ", Name, " skipping - Phase: ", P, " Path: ", Path);
   skip.


+actionID(X):true <-
    .my_name(Name);
    ?phase(P);
    ?current_path(Path);
    if (map_initialized(true) & .empty(Path)) {
        if (has_target_dispenser(TX,TY,Type)) {
            // Verify dispenser at target location
            ?current_pos(CX,CY);
            if (not local_map(CX,CY,Type)) {
                .print("Agent ", Name, " reached target but dispenser not found, replanning");
                -has_target_dispenser(TX,TY,Type);
                !switch_to_planning_mode;
                !plan_next_move;
            };
        } elif (not phase(planning)) {
            !switch_to_planning_mode;
            .print("switching to planning mode");
            ?current_pos(MyX,MyY);
            .print("Agent ", Name, " at ", MyX, ",", MyY, " finished path, switching to planning");
            !plan_next_move;
        };
    } else {
        .print("Agent ", Name, " has path or target dispenser");
        .print("Agent ", Name, " skipping - Phase: ", P, " Path: ", Path);
    };
    skip.


// Update path finding to look for both dispenser types
+!find_closest_dispenser : true <-
  ?current_pos(MyX,MyY);
  .findall(
       [X,Y,Type],
       (local_map(X,Y,Type) & (Type == db0 | Type == db1)),
       Dispensers
   );
  .findall([X,Y],
       (local_map(X,Y,ob) | local_map(X,Y,a)),
       Blocked
   );
  !select_nearest_dispenser(Dispensers, Blocked, MyX, MyY).


+!select_nearest_dispenser([], _, _, _) <- true.
+!select_nearest_dispenser(Dispensers, Blocked, MyX, MyY) : true <-
  .findall([D,X,Y,Type],
       (.member([X,Y,Type], Dispensers) & D = math.abs(X-MyX) + math.abs(Y-MyY)),
       AccessibleDispensers);
   if (not .empty(AccessibleDispensers)) {
      ?find_minimum_distance(AccessibleDispensers, [_,TargetX,TargetY,Type]);
      !generate_path(MyX, MyY, TargetX, TargetY, Blocked, [], Path);
       -+current_path(Path)
   }.


// Simplified path generation with deadend handling
+!generate_path(X, Y, TargetX, TargetY, Blocked, CurrentPath, Path) : true <-
   if (X == TargetX & Y == TargetY) {
      .reverse(CurrentPath, Path);
      .print("Path generation complete. Generated path: ", Path);
   } else {
       if (X < TargetX & not .member([X+1,Y], Blocked)) {
          !generate_path(X+1, Y, TargetX, TargetY, Blocked, [e|CurrentPath], Path)
       } elif (X > TargetX & not .member([X-1,Y], Blocked)) {
          !generate_path(X-1, Y, TargetX, TargetY, Blocked, [w|CurrentPath], Path)
       } elif (Y < TargetY & not .member([X,Y+1], Blocked)) {
          !generate_path(X, Y+1, TargetX, TargetY, Blocked, [s|CurrentPath], Path)
       } elif (Y > TargetY & not .member([X,Y-1], Blocked)) {
          !generate_path(X, Y-1, TargetX, TargetY, Blocked, [n|CurrentPath], Path)
       } else {
           Path = [];
          .print("No valid path found, returning empty path");
       };
   }.


// Helper plan to find minimum distance
+?find_minimum_distance([], Min) : true <- Min = [999,0,0,none].
+?find_minimum_distance([[D,X,Y,Type]|Rest], Min) : true <-
  ?find_minimum_distance(Rest, RestMin);
   RestMin = [MinD,_,_,_];
   if (D < MinD) {
       Min = [D,X,Y,Type];
   } else {
       Min = RestMin;
   }.


// Atomic belief updates
@atomic_pos_update[atomic]
+!update_position(Dir) : current_pos(CX,CY) <-
    if (Dir = n) { -+current_pos(CX,CY-1) }
    elif (Dir = s) { -+current_pos(CX,CY+1) }
    elif (Dir = e) { -+current_pos(CX+1,CY) }
    elif (Dir = w) { -+current_pos(CX-1,CY) }.

@atomic_path_update[atomic]
+!update_path(NewPath) <-
    -current_path(_);
    +current_path(NewPath).

@atomic_failed_moves[atomic]
+!update_failed_moves(Action) : failed_moves(F) <-
    if (Action = increment) {
        -+failed_moves(F+1);
    } elif (Action = reset) {
        -+failed_moves(0);
    }.

@atomic_phase_update[atomic]
+!update_phase(NewPhase) <-
    -+phase(NewPhase).

// Move goal system
+actionID(X) : phase(moving) & current_path([H|T]) <-
    !move(H);
    !update_path(T).

// Basic move goals
+!move(Direction) <-
    move(Direction);
    !update_position(Direction).

// Handle move failures
-!move(Direction)[error(failed_path)] <-
    .my_name(Name);
    ?current_pos(CX,CY);
    ?failed_moves(F);
    if (F < 3) {
        !update_failed_moves(increment);
        .print("Agent ", Name, " move blocked at ", CX, ",", CY, " waiting (attempt ", F+1, ")");
        .wait(500);  // Short wait before retry
        !move(Direction);
    } else {
        !update_failed_moves(reset);
        !update_phase(planning);
        !update_path([]);
        !plan_next_move;
    }.

