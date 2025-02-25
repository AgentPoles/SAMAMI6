//slow with exploration issues
// Initial beliefs and rules
current_pos(0,0).
phase(initial_mapping).
mapping_complete(false).
exploring(false).
has_target(false).
movement_failed(false).
initialization_steps(3).
exploration_limit(10).  // Steps before rechecking for dispensers
exploration_steps(0).  // Add this

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

/* Phase 1: Initial Mapping */
+step(S) : phase(initial_mapping) <- 
    ?initialization_steps(WaitSteps);
    if (S >= WaitSteps) {
        -+phase(planning);
        -+mapping_complete(true);
        !start_planning_phase;
    }.

/* Phase 2: Planning Phase */
+!start_planning_phase : true <-
    .print("Starting planning phase");
    !find_closest_dispenser.

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
    if (.empty(Dispensers)) {
        .print("No dispensers found, entering exploration phase");
        -+phase(exploration);
        -+exploring(true);
        !start_exploration;
    } else {
        !select_nearest_dispenser(Dispensers, Blocked, MyX, MyY);
    }.

/* Phase 3: Exploration Phase */
+!start_exploration : exploring(true) <-
    ?exploration_steps(S);
    -+exploration_steps(S+1);
    !find_unexplored_direction.

+!find_unexplored_direction : true <-
    ?current_pos(CX,CY);
    if (not local_map(CX,CY-1,ob) & not local_map(CX,CY-1,a)) {
        -+current_path([n]);
        -+phase(movement);
    } elif (not local_map(CX+1,CY,ob) & not local_map(CX+1,CY,a)) {
        -+current_path([e]);
        -+phase(movement);
    } elif (not local_map(CX,CY+1,ob) & not local_map(CX,CY+1,a)) {
        -+current_path([s]);
        -+phase(movement);
    } elif (not local_map(CX-1,CY,ob) & not local_map(CX-1,CY,a)) {
        -+current_path([w]);
        -+phase(movement);
    } else {
        -+phase(planning);
        -+exploring(false);
    }.

+!generate_exploration_path(CX, CY, KnownCells, Path) : true <-
    if (not .member([CX,CY-1], KnownCells) & not local_map(CX,CY-1,ob) & not local_map(CX,CY-1,a)) {
        Path = [n];
    } elif (not .member([CX+1,CY], KnownCells) & not local_map(CX+1,CY,ob) & not local_map(CX+1,CY,a)) {
        Path = [e];
    } elif (not .member([CX,CY+1], KnownCells) & not local_map(CX,CY+1,ob) & not local_map(CX,CY+1,a)) {
        Path = [s];
    } elif (not .member([CX-1,CY], KnownCells) & not local_map(CX-1,CY,ob) & not local_map(CX-1,CY,a)) {
        Path = [w];
    } else {
        !find_least_explored_direction(CX, CY, Path);
    }.

+!find_least_explored_direction(CX, CY, Path) : true <-
    .findall([X,Y], local_map(X,Y,_), KnownCells);
    .count(local_map(CX-1,_,_), WCount);
    .count(local_map(CX+1,_,_), ECount);
    .count(local_map(_,CY-1,_), NCount);
    .count(local_map(_,CY+1,_), SCount);
    if (WCount <= ECount & WCount <= NCount & WCount <= SCount & not local_map(CX-1,CY,ob)) {
        Path = [w];
    } elif (ECount <= WCount & ECount <= NCount & ECount <= SCount & not local_map(CX+1,CY,ob)) {
        Path = [e];
    } elif (NCount <= WCount & NCount <= ECount & NCount <= SCount & not local_map(CX,CY-1,ob)) {
        Path = [n];
    } elif (not local_map(CX,CY+1,ob)) {
        Path = [s];
    } else {
        Path = [];
    }.

/* Phase 4: Movement Phase */
+actionID(X) : phase(initial_mapping) <- 
    skip.

+actionID(X) : phase(exploration) <-
    !find_unexplored_direction.

+actionID(X) : phase(planning) <-
    !start_planning_phase.

+actionID(X) : phase(movement) & current_path([H|T]) <-
    ?current_pos(CX,CY);
    move(H);
    if (H = n) { -+current_pos(CX,CY-1) }
    elif (H = s) { -+current_pos(CX,CY+1) }
    elif (H = e) { -+current_pos(CX+1,CY) }
    elif (H = w) { -+current_pos(CX-1,CY) };
    -+current_path(T);
    if (T == []) {  // Immediately handle empty path
        if (exploring(true)) {
            ?exploration_steps(Steps);
            -+exploration_steps(Steps+1);
            ?exploration_limit(Limit);
            if (Steps >= Limit) {
                -+phase(planning);
                -+exploring(false);
                -+exploration_steps(0);
            } else {
                -+phase(exploration);
            }
        } else {
            -+phase(planning);
        }
    }.

/* Phase 5: Revaluation Phase */
+actionID(X) : phase(movement) & current_path([]) <-
    ?current_pos(CX,CY);
    .my_name(Name);
    .print("Agent ", Name, " completed movement at (", CX, ",", CY, ")");
    !status_report;
    !enter_revaluation_phase.

+!enter_revaluation_phase : exploring(true) <-
    ?exploration_limit(Limit);
    ?exploration_steps(Steps);
    if (Steps >= Limit) {
        .print("Exploration limit reached, returning to planning");
        -+phase(planning);
        -+exploring(false);
        -+exploration_steps(0);  // Reset counter
        !start_planning_phase;
    } else {
        -+phase(exploration);
        !start_exploration;
    }.

+!enter_revaluation_phase : has_target(true) <-
    -+has_target(false);
    -+phase(planning);
    !start_planning_phase.

/* Phase 6: Recovery Phase */
+lastActionResult(failed) : phase(movement) <-
    -+movement_failed(true);
    ?current_pos(CX,CY);
    .print("Movement failed at position: (", CX, ",", CY, ")");
    !handle_movement_failure.

+!handle_movement_failure : true <-
    -+current_path([]);
    -+phase(planning);
    -+movement_failed(false);
    !start_planning_phase.

/* Status Reporting */
+!status_report : true <-
    ?current_pos(CX,CY);
    .my_name(Name);
    .findall([PX,PY,db0], local_map(PX,PY,db0), Db0);
    .findall([PX,PY,db1], local_map(PX,PY,db1), Db1);
    .findall([PX,PY,ob], local_map(PX,PY,ob), Obstacles);
    .findall([PX,PY,goal], local_map(PX,PY,goal), Goals);
    .findall([PX,PY,a], local_map(PX,PY,a), Agents);
    .print("Agent ", Name, " status:");
    .print("  - Position: (", CX, ",", CY, ")");
    .print("  - Phase: ", phase(Phase));
    .print("  - B0 Dispensers: ", Db0);
    .print("  - B1 Dispensers: ", Db1);
    .print("  - Obstacles: ", Obstacles);
    .print("  - Goals: ", Goals);
    .print("  - Other Agents: ", Agents).

// Default action
+actionID(X) : true <- 
    skip.

// Add select_nearest_dispenser plan
+!select_nearest_dispenser([], _, _, _) <-
    .print("No dispensers found, entering exploration phase");
    -+phase(exploration);
    -+exploring(true);
    !start_exploration.

+!select_nearest_dispenser(Dispensers, Blocked, MyX, MyY) : true <-
    .findall(
        [D,X,Y,Type],
        (.member([X,Y,Type], Dispensers) & 
         D = math.abs(X-MyX) + math.abs(Y-MyY)),
        Distances
    );
    .min(Distances, [_,TargetX,TargetY,_]);
    !generate_path(MyX, MyY, TargetX, TargetY, Blocked, [], Path);
    -+current_path(Path);
    -+has_target(true);
    -+phase(movement).

// Add path generation plan
+!generate_path(X, Y, TargetX, TargetY, Blocked, CurrentPath, Path) : true <-
    if (X == TargetX & Y == TargetY) {
        .reverse(CurrentPath, Path)
    } else {
        if (X < TargetX & not .member([X+1,Y], Blocked) & not local_map(X+1,Y,ob) & not local_map(X+1,Y,a)) {
            !generate_path(X+1, Y, TargetX, TargetY, Blocked, [e|CurrentPath], Path)
        } elif (X > TargetX & not .member([X-1,Y], Blocked) & not local_map(X-1,Y,ob) & not local_map(X-1,Y,a)) {
            !generate_path(X-1, Y, TargetX, TargetY, Blocked, [w|CurrentPath], Path)
        } elif (Y < TargetY & not .member([X,Y+1], Blocked) & not local_map(X,Y+1,ob) & not local_map(X,Y+1,a)) {
            !generate_path(X, Y+1, TargetX, TargetY, Blocked, [s|CurrentPath], Path)
        } elif (Y > TargetY & not .member([X,Y-1], Blocked) & not local_map(X,Y-1,ob) & not local_map(X,Y-1,a)) {
            !generate_path(X, Y-1, TargetX, TargetY, Blocked, [n|CurrentPath], Path)
        } else {
            Path = [];
            .print("No path found to target")
        }
    }.