/// PAUL OAMEN PLAY GROUND// Initial beliefs and rules

///SUCESSFULLY EXECUTED MOVE TO AGENT POSITION (PATH CLEARING COMMENTED OUT)
current_step(0).
lastAction(none).
current_pos(0,0).  // Our position relative to origin
map_initialized(false).
initialization_steps(3).  // Wait these many steps before planning
has_target(false).  // Track if we have a target
current_path([]).   // Store our current path

// Manhattan distance calculation
manhattan_distance(X1,Y1,X2,Y2,D) :- D = math.abs(X2-X1) + math.abs(Y2-Y1).

/* Phase 1: Initialization and Map Building */

// Step handler - different behavior based on state
+step(S) : not map_initialized(true) <- 
    ?initialization_steps(WaitSteps);
    if (S >= WaitSteps) {
        -+map_initialized(true);
        !start_planning_phase;
    } else {
        -+current_step(S);
        !update_local_map;
    }.

// After initialization, only execute moves if we have a target
+step(S) : map_initialized(true) & has_target(true) <- 
    -+current_step(S);
    !update_local_map;
    ?current_path([H|T]);
    !execute_move(H, T).

// After initialization, plan if we don't have a target
+step(S) : map_initialized(true) & not has_target(true) <- 
    -+current_step(S);
    !update_local_map;
    !start_planning_phase.

// Update local map with new perceptions
+!update_local_map : true <-
    ?current_pos(CX,CY);
    .findall([X,Y,Type], local_map(X,Y,Type), Map);
    .my_name(Name);
    .print(Name, " at step ", S, " knows: ", Map).

// Add things to local_map relative to original position
+thing(X,Y,dispenser,Type)[source(percept)] : true <-
    ?current_pos(CX,CY);
    AbsX = CX + X;
    AbsY = CY + Y;
    ?current_step(S);
    +local_map(AbsX,AbsY,Type)[step(S)].

+thing(X,Y,entity,Type)[source(percept)] : true <-
    ?current_pos(CX,CY);
    AbsX = CX + X;
    AbsY = CY + Y;
    ?current_step(S);
    +local_map(AbsX,AbsY,"A")[step(S)].

+obstacle(X,Y)[source(percept)] : true <-
    ?current_pos(CX,CY);
    AbsX = CX + X;
    AbsY = CY + Y;
    ?current_step(S);
    +local_map(AbsX,AbsY,ob)[step(S)].

/* Phase 2: Planning */

// Start planning phase after initialization
+!start_planning_phase : true <-
    .print("Map initialized, starting planning phase");
    !find_closest_dispenser.

// Find closest dispenser and plan path
+!find_closest_dispenser : true <-
    ?current_pos(MyX,MyY);
    // Get only b0 and b1 dispensers
    .findall(
        [X,Y,Type], 
        (local_map(X,Y,Type) & (Type == b0 | Type == b1)), 
        Dispensers
    );
    // Get obstacles
    .findall([X,Y], 
        (local_map(X,Y,ob) | (local_map(X,Y,"A") & (X \== 0 | Y \== 0))), 
        Blocked
    );
    .print("Current position: (", MyX, ",", MyY, ")");
    .print("Found b0/b1 dispensers: ", Dispensers);
    .print("Found blocked positions: ", Blocked);
    !select_nearest_dispenser(Dispensers, Blocked, MyX, MyY).

// Select nearest accessible dispenser
+!select_nearest_dispenser([], _, _, _) <- .print("No b0/b1 dispensers found").
+!select_nearest_dispenser(Dispensers, Blocked, MyX, MyY) : true <-
    .findall(
        [D,X,Y,Type],
        (
            .member([X,Y,Type], Dispensers) &
            manhattan_distance(MyX,MyY,X,Y,D) &
            not .member([X,Y], Blocked)
        ),
        AccessibleDispensers
    );
    .print("Accessible dispensers with distances: ", AccessibleDispensers);
    if (not .empty(AccessibleDispensers)) {
        ?find_minimum_distance(AccessibleDispensers, [_,TargetX,TargetY,Type]);
        .print("Selected target: (", TargetX, ",", TargetY, ") of type ", Type);
        !plan_path(TargetX, TargetY, Blocked);
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

/* Phase 3: Path Planning and Execution */

// Plan optimal path to target
+!plan_path(TargetX, TargetY, Blocked) : true <-
    ?current_pos(MyX,MyY);
    !generate_path(MyX, MyY, TargetX, TargetY, Blocked, [], Path);
    .print("Generated path: ", Path);
    -+current_path(Path);
    -+has_target(true);
    -+target_pos(TargetX, TargetY);
    .print("Path stored, ready to move").

// Generate path avoiding obstacles
+!generate_path(X, Y, TargetX, TargetY, Blocked, CurrentPath, Path) : true <-
    if (X == TargetX & Y == TargetY) {
        .reverse(CurrentPath, Path);
        .print("Path found: ", Path);
    } else {
        if (X < TargetX & not .member([X+1,Y], Blocked)) {
            !generate_path(X+1, Y, TargetX, TargetY, Blocked, [e|CurrentPath], Path)
        } elif (X > TargetX & not .member([X-1,Y], Blocked)) {
            !generate_path(X-1, Y, TargetX, TargetY, Blocked, [w|CurrentPath], Path)
        } elif (Y < TargetY & not .member([X,Y+1], Blocked)) {
            !generate_path(X, Y+1, TargetX, TargetY, Blocked, [s|CurrentPath], Path)
        } elif (Y > TargetY & not .member([X,Y-1], Blocked)) {
            !generate_path(X, Y-1, TargetX, TargetY, Blocked, [n|CurrentPath], Path)
        }
    }.

// Execute single move and update path
+!execute_move(Move, Rest) : true <-
    .print("Executing move: ", Move);
    ?current_pos(X,Y);
    if (Move = n) { 
        move(n);
        -+current_pos(X,Y-1);
        .print("Moving north to ", X, ",", Y-1)
    }
    elif (Move = s) { 
        move(s);
        -+current_pos(X,Y+1);
        .print("Moving south to ", X, ",", Y+1)
    }
    elif (Move = e) { 
        move(e);
        -+current_pos(X+1,Y);
        .print("Moving east to ", X+1, ",", Y)
    }
    elif (Move = w) { 
        move(w);
        -+current_pos(X-1,Y);
        .print("Moving west to ", X-1, ",", Y)
    };
    -+current_path(Rest).

// When path is empty, clear target
+current_path([]) : true <-
    .print("Path completed, clearing target").

// Handle movement failures
+lastActionResult(failed) : true <-
    .print("Movement failed, clearing target and path");
    -+has_target(false);
    -+current_path([]).