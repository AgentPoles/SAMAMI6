/* Plans */
//submitting the block
+step(X):task(TaskID,_,_,[req(0,1,Type)]) & attached(0,1) & goal(0,0) <- submit(TaskID).

//rotating the block
+step(X): goal(0,0) & attached(1,0) <-  rotate(cw).
+step(X): goal(0,0) & attached(-1,0) <- rotate(ccw).
+step(X): goal(0,0) & attached(0,-1) <- rotate(cw).
      
//moving towards the goal
+step(X) : attached(_,_) & goal(A,B) & (A > 0) & (B == 0) <- move(e).
+step(X) : attached(_,_) & goal(A,B) & (B > 0) & (A == 0) <- move(s).  
+step(X) : attached(_,_) & goal(A,B) & (A < 0) & (B == 0) <- move(w).
+step(X) : attached(_,_) & goal(A,B) & (B < 0) & (A == 0) <- move(n).


//moving randomly after attaching
+step(X) : true & attached(_,_) <- !move_random.

//attaching blocks
+step(X) : thing(1,0,block,_) <- attach(e).
+step(X) : thing(0,1,block,_) <- attach(s).
+step(X) : thing(-1,0,block,_) <- attach(w).
+step(X) : thing(0,-1,block,_) <- attach(n).

//avoiding obstacles for single blocks
+step(X) : thing(1,0,obstacle,_) <- move(n).
+step(X) : thing(0,1,obstacle,_) <- move(e).
+step(X) : thing(-1,0,obstacle,_) <- move(s).
+step(X) : thing(0,-1,obstacle,_) <- move(w).

//requesting blocks
+step(X) : thing(1,0,dispenser,_) <- request(e).
+step(X) : thing(0,1,dispenser,_) <- request(s).
+step(X) : thing(-1,0,dispenser,_) <- request(w).
+step(X) : thing(0,-1,dispenser,_) <- request(n).

//magnetic exploration
+step(X) : thing(A,B,dispenser,_) & (A == 0) & (B < 0) <- move(n).
+step(X) : thing(A,B,dispenser,_) & (A < 0) & (B == 0) <- move(w).
+step(X) : thing(A,B,dispenser,_) & (A > 0) & (B == 0) <- move(e).
+step(X) : thing(A,B,dispenser,_) & (A == 0) & (B > 0) <- move(s).

// exploration
+step(X) : true <- !move_random.

//exploration function
+!move_random <- 
    .random([n,s,e,w], Dir);
    move(Dir).

// Helper plan to evaluate direction safety
+!check_direction(X, Y, Result) <-
    // Check immediate and diagonal obstacles
    if (thing(X,Y,obstacle,_) | thing(X,Y,entity,_)) {
        Result = blocked;
    } elif (thing(X+1,Y,obstacle,_) | thing(X-1,Y,obstacle,_) |
            thing(X,Y+1,obstacle,_) | thing(X,Y-1,obstacle,_)) {
        Result = risky;
    } else {
        Result = safe;
    }.

// Smart dispenser approach with obstacle avoidance
+step(_) : thing(DX,DY,dispenser,_) & not attached(_,_) <- 
    // Check all possible approach directions
    !check_direction(DX+1,DY, EastStatus);
    !check_direction(DX-1,DY, WestStatus);
    !check_direction(DX,DY+1, SouthStatus);
    !check_direction(DX,DY-1, NorthStatus);
    
    if (DX > 0 & EastStatus == safe) {
        move(e);
    } elif (DX < 0 & WestStatus == safe) {
        move(w);
    } elif (DY > 0 & SouthStatus == safe) {
        move(s);
    } elif (DY < 0 & NorthStatus == safe) {
        move(n);
    } else {
        // If direct paths are blocked, try diagonal approach
        if ((DX > 0 & DY > 0) & SouthStatus == safe) {
            move(s);
        } elif ((DX > 0 & DY < 0) & NorthStatus == safe) {
            move(n);
        } elif ((DX < 0 & DY > 0) & SouthStatus == safe) {
            move(s);
        } elif ((DX < 0 & DY < 0) & NorthStatus == safe) {
            move(n);
        } else {
            // If all approaches are risky, find least risky direction
            !find_escape_direction(DX, DY, SafeDir);
            move(SafeDir);
        }.

// Find least risky direction considering target
+!find_escape_direction(TargetX, TargetY, Dir) <-
    .findall([Risk,D], 
        (
            .member(D, [n,s,e,w]) &
            !calculate_direction_risk(D, TargetX, TargetY, Risk)
        ),
        Risks
    );
    .min(Risks, [_,Dir]).

// Calculate risk score for a direction
+!calculate_direction_risk(Dir, TargetX, TargetY, Risk) <-
    if (Dir = n & thing(0,-1,obstacle,_)) {
        Risk = 1000;
    } elif (Dir = s & thing(0,1,obstacle,_)) {
        Risk = 1000;
    } elif (Dir = e & thing(1,0,obstacle,_)) {
        Risk = 1000;
    } elif (Dir = w & thing(-1,0,obstacle,_)) {
        Risk = 1000;
    } else {
        // Lower risk for directions towards target
        if (Dir = e & TargetX > 0) {
            Risk = 1;
        } elif (Dir = w & TargetX < 0) {
            Risk = 1;
        } elif (Dir = n & TargetY < 0) {
            Risk = 1;
        } elif (Dir = s & TargetY > 0) {
            Risk = 1;
        } else {
            Risk = 5;
        };
    }.

// Request blocks when adjacent to dispenser
+step(_) : thing(1,0,dispenser,_) & not attached(_,_) <- request(e).
+step(_) : thing(0,1,dispenser,_) & not attached(_,_) <- request(s).
+step(_) : thing(-1,0,dispenser,_) & not attached(_,_) <- request(w).
+step(_) : thing(0,-1,dispenser,_) & not attached(_,_) <- request(n).

// Default exploration
+step(X) : true <- !move_random.



