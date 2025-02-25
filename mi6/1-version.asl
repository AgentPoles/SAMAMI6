/* Initial beliefs and rules */
random_dir(DirList,RandomNumber,Dir) :- (RandomNumber <= 0.25 & .nth(0,DirList,Dir)) | (RandomNumber <= 0.5 & .nth(1,DirList,Dir)) | (RandomNumber <= 0.75 & .nth(2,DirList,Dir)) | (.nth(3,DirList,Dir)).

/* Initial goals */

!start.

/* Plans */

//submitting the block
+step(X):task(TaskID,_,_,[req(0,1,Type)]) & attached(0,1) & goal(0,0) <- submit(TaskID).

//rotating the block
+step(X): goal(0,0) & attached(1,0) <-
    .print("Rotating the block");
    rotate(cw).

+step(X): goal(0,0) & attached(-1,0) <-
    .print("Rotating the block");
    rotate(ccw).

+step(X): goal(0,0) & attached(0,-1) <-
    .print("Rotating the block");
    rotate(cw).
        

//moving towards the goal
+step(X) : attached(_,_) & goal(A,B) & (A > 0) & (B == 0) <- 
    .print("Moving to the east with attached block");
      move(e).

+step(X) : attached(_,_) & goal(A,B) & (B > 0) & (A == 0) <- 
    .print("Moving to the south with attached block");
      move(s).    

+step(X) : attached(_,_) & goal(A,B) & (A < 0) & (B == 0) <- 
    .print("Moving to the west with attached block");
      move(w). 

+step(X) : attached(_,_) & goal(A,B) & (B < 0) & (A == 0) <- 
    .print("Moving to the north with attached block");
      move(n). 


//moving randomly after attaching
+step(X) : true & attached(_,_) <- 
    .print("Moving randomly after attaching");
    !move_random.

//attaching blocks
+step(X) : thing(1,0,block,_) <- 
    .print("Attaching block to the east");
	attach(e).

+step(X) : thing(0,1,block,_) <- 
    .print("Attaching block to the south");
	attach(s).

+step(X) : thing(-1,0,block,_) <- 
    .print("Attaching block to the west");
	attach(w).

+step(X) : thing(0,-1,block,_) <- 
    .print("Attaching block to the north");
	attach(n).


//requesting blocks
+step(X) : thing(1,0,dispenser,_) <- 
    .print("Requesting dispenser to the east");
	request(e).

+step(X) : thing(0,1,dispenser,_) <- 
    .print("Requesting dispenser to the south");
	request(s).

+step(X) : thing(-1,0,dispenser,_) <- 
    .print("Requesting dispenser to the west");
	request(w).

+step(X) : thing(0,-1,dispenser,_) <- 
    .print("Requesting dispenser to the north");
	request(n).

// initial moving random
+step(X) : true <- 
	.print("Determining my action");
	!move_random.


+!move_random : .random(RandomNumber) & random_dir([n,s,e,w],RandomNumber,Dir)
<-	move(Dir).
