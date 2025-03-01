/* Plans */
+goal(X,Y)[source(percept)] : true <-
    helpermodels.AddGoal(X, Y).

+obstacle(X,Y)[source(percept)] : true <-
    helpermodels.AddObstacle(X, Y).

+thing(X,Y,dispenser,Detail)[source(percept)] : true <-
    helpermodels.AddDispenser(X, Y, Detail).

+thing(X,Y,entity,_)[source(percept)] : true <-
    helpermodels.AddOtherAgents(X, Y).

+step(X):task(TaskID,_,_,[req(0,1,Type)]) & attached(0,1) & goal(0,0) <- submit(TaskID).

//rotating the block
+step(X): goal(0,0) & attached(1,0) <-  rotate(cw).
+step(X): goal(0,0) & attached(-1,0) <- rotate(ccw).
+step(X): goal(0,0) & attached(0,-1) <- rotate(cw).
      

//moving randomly after attaching
+step(X) : true & attached(_,_) <- !move_random(goal).

//attaching blocks
+step(X) : thing(1,0,block,_) & thing(1,0,dispenser,_) <- attach(e).
+step(X) : thing(0,1,block,_) & thing(0,1,dispenser,_) <- attach(s).
+step(X) : thing(-1,0,block,_) & thing(-1,0,dispenser,_) <- attach(w).
+step(X) : thing(0,-1,block,_) & thing(0,-1,dispenser,_) <- attach(n).

//requesting blocks
+step(X) : thing(1,0,dispenser,_) & not attached(_,_) <- request(e).
+step(X) : thing(0,1,dispenser,_) & not attached(_,_) <- request(s).
+step(X) : thing(-1,0,dispenser,_) & not attached(_,_) <- request(w).
+step(X) : thing(0,-1,dispenser,_) & not attached(_,_) <- request(n).

+step(X) : true <- 
	!move_random(dispenser).

+actionResult(move,failed_path)[source(percept)] : true <-
    .print("Move failed: path blocked");
    helpermodels.AddMovementFailure(failed_path).

+actionResult(move,failed_forbidden)[source(percept)] : true <-
    .print("Move failed: boundary detected");
    helpermodels.AddMovementFailure(failed_forbidden).

+!move_random(Target): helpermodels.RequestGuidance(Target,1,Dir)
<-  .print("Moving towards dispenser: ", Dir);
    move(Dir);
    helpermodels.UpdateMovement(Dir).

+!move_random(_)
<-  .print("No guidance available, moving north");
    move(n);
    helpermodels.UpdateMovement(n).
