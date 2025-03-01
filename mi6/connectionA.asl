// Combined action result handling with parameters


// Debug all action results with more detail
+lastActionResult(R)[source(percept)] : lastActionParams([Dir])[source(percept)] <-
    .print("DEBUG: Action result received: ", R, " with direction ", Dir);
    if (R == failed_forbidden) {
        .print("BOUNDARY DETECTED: Failed moving ", Dir);
        helpermodels.AddMovementFailure(failed_forbidden, Dir);
    } elif (R == failed_path) {
        .print("PATH BLOCKED: Failed moving ", Dir);
        helpermodels.AddMovementFailure(failed_path, Dir);
    } elif (R == success) {
        .print("Move succeeded in direction: ", Dir);
    }.

+step(X) : true & attached(0,1) <- !move_random(goal,2,s).
+step(X) : true & attached(0,-1) <- !move_random(goal,2,n).
+step(X) : true & attached(1,0) <- !move_random(goal,2,e).
+step(X) : true & attached(-1,0) <- !move_random(goal,2,w).

//attaching blocks
+step(X) :  thing(1,0,dispenser,_) & thing(1,0,block,_) <- attach(e).
+step(X) :  thing(0,1,dispenser,_) & thing(0,1,block,_) <- attach(s).
+step(X) :  thing(-1,0,dispenser,_) & thing(-1,0,block,_) <- attach(w).
+step(X) :  thing(0,-1,dispenser,_) & thing(0,-1,block,_) <- attach(n).

//requesting blocks
+step(X) : thing(1,0,dispenser,_) & not attached(_,_) <- request(e).
+step(X) : thing(0,1,dispenser,_) & not attached(_,_) <- request(s).
+step(X) : thing(-1,0,dispenser,_) & not attached(_,_) <- request(w).
+step(X) : thing(0,-1,dispenser,_) & not attached(_,_) <- request(n).

// Step handling with debug
+step(X) : true <-
    .print("Step ", X, " started for agent ", .my_name);
    !move_random(dispenser,1,"null").
    
// Movement with debug
+!move_random(Target,Size,BlockDir): helpermodels.RequestGuidance(Target,Size,BlockDir,Dir)
<- 
    .print("DEBUG: Attempting to move towards ", Target, " in direction ", Dir);
    move(Dir);
    .print("DEBUG: Move action sent");
    helpermodels.UpdateMovement(Dir).

+!move_random(_)
<- 
    .print("DEBUG: No guidance, attempting to move north");
    move(n);
    .print("DEBUG: North move action sent");
    helpermodels.UpdateMovement(n).

// Error handling with debug
-!move_random(Target)[error(Error), error_msg(Msg)] : true <-
    .print("ERROR in move_random: ", Error, " - ", Msg);
    !move_random(Target).

// Catch-all for unhandled failures
-!G[error(Error), error_msg(Msg)] : true <- 
    .print("UNHANDLED ERROR: ", G, " failed with ", Error, " - ", Msg).

