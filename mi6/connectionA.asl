// Initial beliefs and rules
current_step(0).
lastAction(none).
myPos(0,0).

// Step handling with belief persistence
+step(S) : true <- 
    ?current_step(CS);
    -+current_step(S);  // Only update step counter
    .findall([X,Y,V], map(X,Y,V), Map);
    .my_name(Name);
    .print(Name, " at step ", S, " knows: ", Map).

// Perception handlers with persistent beliefs
+thing(X,Y,dispenser,Type)[source(percept)] : true <- 
    ?current_step(S);
    +map(X,Y,Type)[step(S)].  // Add with step annotation

+thing(X,Y,block,Type)[source(percept)] : true <- 
    ?current_step(S);
    +map(X,Y,Type)[step(S)].

+thing(X,Y,entity,Type)[source(percept)] : true <- 
    ?current_step(S);
    +map(X,Y,Type)[step(S)].

+obstacle(X,Y)[source(percept)] : true <- 
    ?current_step(S);
    +map(X,Y,ob)[step(S)].

+goal(X,Y)[source(percept)] : true <- 
    ?current_step(S);
    +map(X,Y,goal)[step(S)].

// Position update
+pos(X,Y)[source(percept)] : true <-
    -+myPos(X,Y).
