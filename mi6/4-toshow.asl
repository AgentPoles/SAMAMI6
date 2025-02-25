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


// exploration
+step(X) : true <- !move_random.

//exploration function
+!move_random <- 
    .random([n,s,e,w], Dir);
    move(Dir).



