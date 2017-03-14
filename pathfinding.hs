procedure CalculateKey(s)
	return [ min(g(s), rhs(s)) + h(s₀, s) + k m,
			 min(g(s), rhs(s)) ];

procedure Initialize()
	U = ∅;
	k m =0;
	for all s∈S
		rhs(s) = g(s) = ∞;
	rhs(s goal) = 0;
	U.Insert(s goal, [h(s₀ ,s goal); 0]);

procedure UpdateVertex(u)
	if( 	 g(u) ≠ rhs ( u) AND u ∈ U) U.Update(u,CalculateKey(u));
	else if( g(u) ≠ rhs ( u) AND u ∉ U) U.Insert(u,CalculateKey(u));
	else if( g(u) = rhs ( u) AND u ∈ U) U.Remove(u);

procedure ComputeShortestPath()
	while ( U.TopKey() < CalculateKey(s₀) OR rhs(s₀) > g(s₀))
		u = U.Top() ;
		k old = U.TopKey() ;
		k new = CalculateKey(u)) ;
		if ( k old < k new)
			U.Update(u,k new);
		else if(g(u) > rhs(u))
			g(u)= rhs(u);
			U.Remove(u) ;
			for all s∈Pred(u)
				if (s ≠ s goal)
					rhs(s) = min( rhs(s), c(s,u) + g(u));
				UpdateVertex ( s) ;
		else
			g old = g(u) ;
			g (u)= ∞ ;
			for all s∈Pred(u) ∪ {u}
				if (rhs(s) = c (s,u)+ g old)
					if (s ≠ s goal)
						rhs(s) = min s′∈Succ(s) (c(s,s′)+ g (s′));
				UpdateVertex (s);

procedure Main ()
	s₊ = s₀ ;
	Initialize () ;
	ComputeShortestPath () ;
	while ( s₀ ≠ s goal)
	/* if ( g ( s₀)= ∞) then there is no known path */
	s₀ = argmin s ∈ Succ(s₀) (c(s₀ ,s′) + g(s′)) ;
	Move to s₀ ;
	Scan graph for changed edge costs;
	if any edge costs changed
		k m = k m + h(s₊ ,s₀);
		s₊ = s₀;
		for all directed edges (u,v) with changed edge costs
			c old = c(u,v) ;
			Update the edge costc ( u,v) ;
			if (c old > c( u,v))
				if ( u ≠ s goal)
					rhs(u) = min(rhs(u), c(u,v) + g(v)) ;
			else if (rhs(u)= c old + g(v))
				if (u ≠ s goal)
					rhs(u) = min s′∈Succ(u) (c(u,s′)+ g(s′)) ;
			UpdateVertex (u) ;
		ComputeShortestPath() ;
