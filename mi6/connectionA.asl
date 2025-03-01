// Combined action result handling with parameters
+lastAction(move)[source(percept)] : 
    lastActionResult(Result)[source(percept)] & 
    lastActionParams([Dir])[source(percept)]
<- 
    .print("DEBUG: Move action ", Dir, " resulted in ", Result);
    if (Result == failed_forbidden) {
        .print("BOUNDARY DETECTED: Failed moving ", Dir);
        helpermodels.AddMovementFailure(failed_forbidden, Dir);
    } elif (Result == failed_path) {
        .print("PATH BLOCKED: Failed moving ", Dir);
        helpermodels.AddMovementFailure(failed_path, Dir);
    } elif (Result == success) {
        .print("Move succeeded in direction: ", Dir);
    }.

// Debug all percepts
+percept(P)[source(percept)] : true <-
    .print("DEBUG: Received percept: ", P).

// Track last action
+lastAction(A)[source(percept)] : true <-
    .print("DEBUG: Last action performed: ", A);
    +my_last_action(A).

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

// Step handling with debug
+step(X) : true <-
    .print("Step ", X, " started for agent ", .my_name);
    !move_random(dispenser).
    
// Movement with debug
+!move_random(Target): helpermodels.RequestGuidance(Target,1,Dir)
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
    .wait(500);
    !move_random(Target).

// Catch-all for unhandled failures
-!G[error(Error), error_msg(Msg)] : true <- 
    .print("UNHANDLED ERROR: ", G, " failed with ", Error, " - ", Msg).

