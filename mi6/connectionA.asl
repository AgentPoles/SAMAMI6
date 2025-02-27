/* Initial beliefs and rules */
random_dir(DirList,RandomNumber,Dir) :- (RandomNumber <= 0.25 & .nth(0,DirList,Dir)) | (RandomNumber <= 0.5 & .nth(1,DirList,Dir)) | (RandomNumber <= 0.75 & .nth(2,DirList,Dir)) | (.nth(3,DirList,Dir)).

/* Initial goals */

!start.

/* Plans */
+goal(X,Y)[source(percept)] : true <-
    helpermodels.AddGoal(X, Y).

+obstacle(X,Y)[source(percept)] : true <-
    helpermodels.AddObstacle(X, Y).


+thing(X,Y,dispenser,Detail)[source(percept)] : true <-
    helpermodels.AddDispenser(X, Y, Detail).

+step(X) : true <-
	.print("Received step percept.").
	
+actionID(X) : true <- 
	.print("Determining my action");
	!move_random.



+!move_random : .random(RandomNumber) & random_dir([n,s,e,w],RandomNumber,Dir)
<-	move(Dir);
    helpermodels.UpdateMovement(Dir).