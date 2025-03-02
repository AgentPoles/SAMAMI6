// Combined action result handling with parameters
/* Plans */
+goal(X,Y)[source(percept)] : true <-
    helpermodels.AddGoal(X, Y).

+obstacle(X,Y)[source(percept)] : true <-
    helpermodels.AddObstacle(X, Y).

+thing(X,Y,dispenser,Detail)[source(percept)] : true <-
    helpermodels.AddDispenser(X, Y, Detail).

+thing(X,Y,entity,_)[source(percept)] : true <-
    helpermodels.AddOtherAgents(X, Y).

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
    }
    .

+step(X):task(TaskID,_,_,[req(0,1,Type)]) & attached(0,1) & goal(0,0) <- submit(TaskID); -carryingBlock.

//rotating the block
+step(X): goal(0,0) & attached(1,0) & carryingBlock  <-  rotate(cw).
+step(X): goal(0,0) & attached(-1,0) & carryingBlock <- rotate(ccw).
+step(X): goal(0,0) & attached(0,-1) & carryingBlock <- rotate(cw).

+step(X) : true & attached(0,1) & carryingBlock <- !move_random(goal,2,s).
+step(X) : true & attached(0,-1) & carryingBlock <- !move_random(goal,2,n).
+step(X) : true & attached(1,0) & carryingBlock <- !move_random(goal,2,e).
+step(X) : true & attached(-1,0) & carryingBlock <- !move_random(goal,2,w).

+step(X) : attached(0,1) & not carryingBlock <- detach(e).
+step(X) : attached(0,-1) & not carryingBlock <- detach(s).
+step(X) : attached(1,0) & not carryingBlock <- detach(w).
+step(X) : attached(-1,0) & not carryingBlock <- detach(n).

//attaching blocks
+step(X) :  not attached(_,_) & thing(1,0,dispenser,_) & thing(1,0,block,_) <- +carryingBlock; attach(e).
+step(X) :  not attached(_,_) & thing(0,1,dispenser,_) & thing(0,1,block,_) <- +carryingBlock; attach(s).
+step(X) :  not attached(_,_) & thing(-1,0,dispenser,_) & thing(-1,0,block,_) <- +carryingBlock; attach(w).
+step(X) :  not attached(_,_) & thing(0,-1,dispenser,_) & thing(0,-1,block,_) <- +carryingBlock; attach(n).

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

