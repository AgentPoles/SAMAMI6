
/* Plans */
+goal(X,Y)[source(percept)] : true <-
    helpermodels.AddGoal(X, Y).

+obstacle(X,Y)[source(percept)] : true <-
    helpermodels.AddObstacle(X, Y).


+thing(X,Y,dispenser,Detail)[source(percept)] : true <-
    helpermodels.AddDispenser(X, Y, Detail).

+thing(X,Y,entity,_)[source(percept)] : true <-
    helpermodels.AddOtherAgents(X, Y).


	
+step(X) : true <- 
	.print("Determining my action");
	!move_random.

+!move_random: attached(_,_) & helpermodels.RequestGuidance(dispenser,1,Dirs) & .nth(0,Dirs,Dir)
<-	move(Dir);
    helpermodels.UpdateMovement(Dir).

+!move_random: helpermodels.RequestGuidance(dispenser,1,Dirs) & .nth(0,Dirs,Dir)
<-	.print("Requesting guidance");
    move(Dir);
    helpermodels.UpdateMovement(Dir).
