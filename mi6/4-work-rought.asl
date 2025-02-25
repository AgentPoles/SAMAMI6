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


// Main planning decision with adjacent position handling
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
    !update_phase(moving).


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


// Modified dispenser pathfinding with adjacent positions
+!plan_dispenser_path(Dispensers, Blocked, MyX, MyY) : true <-
    .my_name(Name);
    !attempt_dispenser_path(Dispensers, Blocked, MyX, MyY, 2).


// Helper plan to calculate and return adjacent positions through unification
+!calculate_adjacent_positions(DispenserX, DispenserY, Blocked, Result) <-
    .findall([AX,AY,Dir],
        (
            // North
            (AX = DispenserX & AY = DispenserY-1 & Dir = s & not .member([AX,AY], Blocked)) |
            // South
            (AX = DispenserX & AY = DispenserY+1 & Dir = n & not .member([AX,AY], Blocked)) |
            // East
            (AX = DispenserX+1 & AY = DispenserY & Dir = w & not .member([AX,AY], Blocked)) |
            // West
            (AX = DispenserX-1 & AY = DispenserY & Dir = e & not .member([AX,AY], Blocked))
        ),
        Result
    ).

// Modified dispenser pathfinding
+!attempt_dispenser_path(Dispensers, Blocked, MyX, MyY, Attempts) : true <-
    .my_name(Name);
    // Filter out dispensers with agents
    .findall([X,Y,Type],
        (.member([X,Y,Type], Dispensers) & not local_map(X,Y,a)),
        UnoccupiedDispensers
    );
    
    // First calculate all adjacent positions
    !calculate_adjacent_positions(X, Y, Blocked, AdjacentPositions);
    
    // Then use the result in findall
    .findall([D,X,Y,Type,AX,AY,Dir],
        (
            .member([X,Y,Type], UnoccupiedDispensers) &
            .member([AX,AY,Dir], AdjacentPositions) &
            D = math.abs(AX-MyX) + math.abs(AY-MyY)
        ),
        AccessibleDispensers
    );
    
    if (not .empty(AccessibleDispensers)) {
        ?find_minimum_distance_with_adjacent(AccessibleDispensers, [_,TargetX,TargetY,Type,AdjX,AdjY,Dir]);
        !generate_path(MyX, MyY, AdjX, AdjY, Blocked, [], GeneratedPath);
        
        if (GeneratedPath \== [] & .length(GeneratedPath, L) & L <= 8) {
            !update_path(GeneratedPath);
            !update_dispenser_target(set, TargetX, TargetY, Type, Dir);
            .print("Agent ", Name, " generated valid path to dispenser adjacent position: ", GeneratedPath);
        } else {
            // Try intermediate path
            DX = AdjX - MyX;
            DY = AdjY - MyY;
            IntermediateX = MyX + (DX div 2);
            IntermediateY = MyY + (DY div 2);
            !generate_path(MyX, MyY, IntermediateX, IntermediateY, Blocked, [], ShorterPath);
            
            if (ShorterPath \== [] & .length(ShorterPath, L2) & L2 <= 4) {
                !update_path(ShorterPath);
                !update_dispenser_target(set, TargetX, TargetY, Type, Dir);
                .print("Agent ", Name, " generated intermediate path to dispenser: ", ShorterPath);
            } else {
                !plan_exploration_path(MyX, MyY);
            }
        }
    } else {
        .print("Agent ", Name, " found no accessible dispensers, falling back to exploration");
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


// Action handling with dispenser awareness
+actionID(X) : phase(moving) & current_path([H|T]) & has_target_dispenser(TX,TY,Type) & target_direction(Dir) & .empty(T) <-
    .my_name(Name);
    ?current_pos(CX,CY);
    .print("Agent ", Name, " at final position ", CX, ",", CY, " ready to request from dispenser in direction ", Dir);
    !update_path([]).
    // Next step will handle the request action.

+actionID(X) : phase(moving) & current_path([H|T]) <-
    !move(H);
    !update_path(T).

// When path is empty and we're at the right position, request from dispenser
+actionID(X) : phase(moving) & .empty(current_path) & has_target_dispenser(TX,TY,Type) & target_direction(Dir) <-
    .my_name(Name);
    ?current_pos(CX,CY);
    .print("Agent ", Name, " requesting block from dispenser in direction ", Dir);
    request(Dir).

// Regular empty path handling
+actionID(X) : phase(moving) & .empty(current_path) <-
    .my_name(Name);
    .print("Agent ", Name, " path complete, switching to planning");
    !update_phase(planning);
    !plan_next_move.


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


// Helper plan to find minimum distance including adjacent position
+?find_minimum_distance_with_adjacent([], Min) : true <- 
    Min = [999,0,0,none,0,0,none].

+?find_minimum_distance_with_adjacent([[D,X,Y,Type,AX,AY,Dir]|Rest], Min) : true <-
    ?find_minimum_distance_with_adjacent(Rest, RestMin);
    RestMin = [MinD,_,_,_,_,_,_];
    if (D < MinD) {
        Min = [D,X,Y,Type,AX,AY,Dir];
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

// Verification of dispenser position with return value
+!verify_dispenser_position(X, Y, Type, Dir, Result) : true <-
    if (Dir = n & thing(0,-1,dispenser,Type)) {
        Result = true;
    } elif (Dir = s & thing(0,1,dispenser,Type)) {
        Result = true;
    } elif (Dir = e & thing(1,0,dispenser,Type)) {
        Result = true;
    } elif (Dir = w & thing(-1,0,dispenser,Type)) {
        Result = true;
    } else {
        Result = false;
    }.

// Modified handler using the verification result
+!handle_request_result(Dir) : true <-
    ?has_target_dispenser(TX,TY,Type);
    !verify_dispenser_position(TX,TY,Type,Dir,IsValid);
    if (not IsValid) {
        .print("Dispenser not found at expected position, replanning");
        !update_dispenser_target(clear,0,0,none,none);
        !update_phase(planning);
        !plan_next_move;
    }.

// Modified move goal system with dispenser awareness
+!move(Direction) : thing(X,Y,dispenser,_) & 
    ((Direction = n & X = 0 & Y = -1) |
     (Direction = s & X = 0 & Y = 1) |
     (Direction = e & X = 1 & Y = 0) |
     (Direction = w & X = -1 & Y = 0)) <-
    .print("Adjacent to dispenser, stopping here");
    !update_path([]).

+!move(Direction) <-
    move(Direction);
    !update_position(Direction).

// Handle move failures with dispenser awareness
-!move(Direction)[error(failed_path)] : has_target_dispenser(TX,TY,Type) <-
    .my_name(Name);
    .print("Agent ", Name, " blocked while approaching dispenser, replanning");
    !update_dispenser_target(clear,0,0,none,none);
    !update_phase(planning);
    !plan_next_move.

-!move(Direction)[error(failed_path)] <-
    .my_name(Name);
    .print("Agent ", Name, " move blocked, replanning");
    !update_phase(planning);
    !update_path([]);
    !plan_next_move.

-!move(Direction)[error(failed_forbidden)] <-
    .my_name(Name);
    .print("Agent ", Name, " hit map boundary - replanning");
    !update_phase(planning);
    !update_path([]);
    !plan_next_move.

-!move(Direction)[error(failed_parameter)] <-
    .print("Invalid move direction: ", Direction).

// Additional atomic belief update for target direction
@atomic_target_direction[atomic]
+!update_target_direction(Dir) <-
    -+target_direction(Dir).

// Modified atomic dispenser update to include direction
@atomic_dispenser_update[atomic]
+!update_dispenser_target(Action, X, Y, Type, Dir) <-
    if (Action = set) {
        -+has_target_dispenser(X,Y,Type);
        -+target_direction(Dir);
    } elif (Action = clear) {
        -has_target_dispenser(_,_,_);
        -target_direction(_);
    }.

