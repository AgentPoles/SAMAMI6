/* Plans */
// Step 1: Find path to nearest dispenser and move
+step(1) : .find_nearest_dispenser <- 
    .print("Path found, moving to nearest dispenser");
    .move_to_nearest_dispenser.

//attaching blocks
+step(X) : thing(1,0,block,_) <- attach(e).
+step(X) : thing(0,1,block,_) <- attach(s).
+step(X) : thing(-1,0,block,_) <- attach(w).
+step(X) : thing(0,-1,block,_) <- attach(n).

// Request blocks when adjacent to dispenser
+step(_) : thing(1,0,dispenser,_) <- request(e).
+step(_) : thing(0,1,dispenser,_) <- request(s).
+step(_) : thing(-1,0,dispenser,_) <- request(w).
+step(_) : thing(0,-1,dispenser,_) <- request(n).

// Smart dispenser approach with obstacle avoidance
+step(_) : thing(DX,DY,dispenser,_) & not attached(_,_) <-
    move_best_random_direction(DX, DY).

// Default random movement if no other plan applies
+step(_) : true <-
    move_best_random_direction(0, 0).



    