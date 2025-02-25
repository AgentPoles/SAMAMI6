// Initial beliefs and rules
current_pos(0,0).
map_initialized(false).
initialization_steps(3).
phase(initial).
current_path([]).  // Initialize empty path

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

// Dispenser pathfinding - keeping existing logic
+!plan_dispenser_path(Dispensers, Blocked, MyX, MyY) : true <-
    .findall([D,X,Y,Type],
        (.member([X,Y,Type], Dispensers) & D = math.abs(X-MyX) + math.abs(Y-MyY)),
        AccessibleDispensers);
    if (not .empty(AccessibleDispensers)) {
        ?find_minimum_distance(AccessibleDispensers, [_,TargetX,TargetY,Type]);
        !generate_path(MyX, MyY, TargetX, TargetY, Blocked, [], Path);
        -+current_path(Path);
        -+has_target_dispenser(TargetX,TargetY,Type);
        .print("Agent ", Name, " generated path dispenser: ", Path);
    }.

// Simple exploration path planning
+!plan_exploration_path(MyX, MyY) : true <-
    .random([-5,-4,-3,-2,-1,1,2,3,4,5], DX);
    .random([-5,-4,-3,-2,-1,1,2,3,4,5], DY);
    TargetX = MyX + DX;
    TargetY = MyY + DY;
    .findall([X,Y], 
        (local_map(X,Y,ob) | local_map(X,Y,a)), 
        Blocked
    );
    !generate_path(MyX, MyY, TargetX, TargetY, Blocked, [], Path);
    -+current_path(Path);
    .print("Agent ", Name, " generated path exploration: ", Path).

// Action handling with safer path checking
+actionID(X) : phase(moving) & current_path([H|T]) <-
    .my_name(Name);
    ?current_pos(CX,CY);
    .print("Agent ", Name, " executing move ", H, " from ", CX, ",", CY, " path remaining: ", T);
    move(H);
    if (H = n) { -+current_pos(CX,CY-1) }
    elif (H = s) { -+current_pos(CX,CY+1) }
    elif (H = e) { -+current_pos(CX+1,CY) }
    elif (H = w) { -+current_pos(CX-1,CY) };
    -+current_path(T).

+actionID(X) : phase(moving) & .empty(current_path) <-
    .my_name(Name);
    ?phase(P);
    ?current_path(Path);
    .print("Agent ", Name, " skipping - Phase: ", P, " Path: ", Path);
    if (has_target_dispenser(TX,TY,Type)) {
        .print("Agent ", Name, " has target dispenser: ", Type, " at (", TX, ",", TY, ")");
    } else {
        .print("Agent ", Name, " has no target dispenser");
    };
    skip.

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
    if(map_initialized(true) & .empty(Path) & not has_target_dispenser(TX,TY,Type) & not phase(planning)) {
          !switch_to_planning_mode;
         .print("switching to planning mode");
         .my_name(Name);
         ?current_pos(MyX,MyY);
         .print("Agent ", Name, " at ", MyX, ",", MyY, " finished path, switching to planning");
         !plan_next_move;
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

// Simplified path generation
+!generate_path(X, Y, TargetX, TargetY, Blocked, CurrentPath, Path) : true <-
    if (X == TargetX & Y == TargetY) {
        .reverse(CurrentPath, Path)
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