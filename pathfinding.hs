-- procedure CalculateKey(s)
-- 	return [ min(g(s), rhs(s)) + h(s₀, s) + k m,
-- 			 min(g(s), rhs(s)) ];

-- procedure Initialize()
-- 	U = ∅;
-- 	k m =0;
-- 	for all s∈S
-- 		rhs(s) = g(s) = ∞;
-- 	rhs(s goal) = 0;
-- 	U.Insert(s goal, [h(s₀ ,s goal); 0]);
-- 
-- procedure UpdateVertex(u)
-- 	if( 	 g(u) ≠ rhs ( u) AND u ∈ U) U.Update(u,CalculateKey(u));
-- 	else if( g(u) ≠ rhs ( u) AND u ∉ U) U.Insert(u,CalculateKey(u));
-- 	else if( g(u) = rhs ( u) AND u ∈ U) U.Remove(u);
-- 
-- procedure ComputeShortestPath()
-- 	while ( U.TopKey() < CalculateKey(s₀) OR rhs(s₀) > g(s₀))
-- 		u = U.Top() ;
-- 		k old = U.TopKey() ;
-- 		k new = CalculateKey(u)) ;
-- 		if ( k old < k new)
-- 			U.Update(u,k new);
-- 		else if(g(u) > rhs(u))
-- 			g(u)= rhs(u);
-- 			U.Remove(u) ;
-- 			for all s∈Pred(u)
-- 				if (s ≠ s goal)
-- 					rhs(s) = min( rhs(s), c(s,u) + g(u));
-- 				UpdateVertex ( s) ;
-- 		else
-- 			g old = g(u) ;
-- 			g(u)= ∞ ;
-- 			for all s∈Pred(u) ∪ {u}
-- 				if (rhs(s) = c (s,u) + g old)
-- 					if (s ≠ s goal)
-- 						rhs(s) = min s′∈Succ(s) (c(s,s′)+ g (s′));
-- 				UpdateVertex (s);

import qualified Data.Vector.Unboxed as V
import qualified Data.PSQueue as U
import Control.Monad.State

xMax = 10 :: Int
yMax = 10 :: Int
costMax = 1000 :: Int

type Location = (Int, Int)
type Cost = Int
type Terrain = V.Vector Cost

 -- Priority ( min(g(s), rhs(s)) + h(s₀, s) + k m, min(g(s), rhs(s)) )
type Priority = (Cost, Cost)
type PathHeap = U.PSQ Location Priority

-- PathState ( Terrain Costs, Cumulative Cost, Estimated Cost, Location Heap )
type PathState = (Terrain, V.Vector Cost, V.Vector Cost, PathHeap)

vpos :: Location -> Int
vpos (x,y) = y * yMax + x

computeShortestPath :: PathHeap -> Location -> Int -> State PathState ()
computeShortestPath pq s0 dK = do
	g_s0 <- g s0
	rhs_s0 <- rhs s0

	g_u <- g u
	rhs_u <- rhs u
	let u_prio' = calcPrio u g_u rhs_u

	if u_prio < (calcPrio s0 g_s0 rhs_s0) || rhs_s0 > g_s0 then

		if u_prio < u_prio' then computeShortestPath (U.insert u u_prio' pq) s0 dK
		else
			if g_u > rhs_u then
				gset u rhs_u

[(x,y) | x <- [-1,0,1], y <- [-1,0,1]]

- 			U.Remove(u) ;
-- 			for all s∈Pred(u)
-- 				if (s ≠ s goal)
-- 					rhs(s) = min( rhs(s), c(s,u) + g(u));
-- 				UpdateVertex ( s) ;

			else return ()
	else return ()

	where
		calcPrio s g_s rhs_s = ( (min g_s rhs_s) + (h s0 s) + dK, min g_s rhs_s )
		(u, u_prio, pq') = maybe ((0,0), (costMax, costMax), pq) (\(kp, pq') -> (U.key kp, U.prio kp, pq')) $ U.minView pq

updateNeighbors goal u _ [] = do return ()
updateNeighbors goal u (x,y) (dx,dy):ts
	| x' >= xMax || y' >= yMax = do return ()
	| x' < 0     || y' < 0     = do return ()
	| u == goal 			   = do return ()
	| otherwise				   = do
	rhsset s $ min rhs s, c s u + g u 
	where
		x' = x + dx
		y' = y + dy
		s = (x', y')

rhs :: Location -> State PathState Cost
rhs s = do
	(_, _, estimated, _) <- get
	return $ estimated V.! (vpos s)
	
rhsset :: Location -> Cost -> State PathState ()
rhsset (x,y) c = do
	(terrain, cumulative, estimated, heap) <- get
	put (terrain, cumulative, estimated V.// [(y * yMax + x, c)], heap)

g :: Location -> State PathState Cost
g s = do
	(_, cumulative, _, _) <- get
	return $ cumulative V.! (vpos s)
	
gset :: Location -> Cost -> State PathState ()
gset (x,y) c = do
	(terrain, cumulative, estimated, heap) <- get
	put (terrain, cumulative V.// [(y * yMax + x, c)], estimated, heap)

c :: Location -> State PathState Cost
c s = do
	(terrain, _, _, _) <- get
	return $ terrain V.! (vpos s)

hook = terrain
	where
		terrain :: V.Vector Int
		terrain = V.generate (xMax * yMax) (\i -> if mod i 3 == 0 || mod i 2 == 1 then 1 else 5)

		cumulative :: V.Vector Int
		cumulative = V.replicate (xMax * yMax) costMax

		estimated :: V.Vector Int
		estimated = V.replicate (xMax * yMax) costMax

h :: Location -> Location -> Int
h start end = 0

printVector :: V.Vector Cost -> IO ()
printVector vector = let
	loop v cnt
		| cnt >= yMax = return ()
		| otherwise = putStrLn (show row) >> loop vec (cnt + 1)
		where (row, vec) = V.splitAt yMax v
	in loop vector 1

-- procedure Main ()
-- 	s₊ = s₀ ;
-- 	Initialize () ;
-- 	ComputeShortestPath () ;
-- 	while ( s₀ ≠ s goal)
-- 	/* if ( g ( s₀)= ∞) then there is no known path */
-- 	s₀ = argmin s ∈ Succ(s₀) (c(s₀ ,s′) + g(s′)) ;
-- 	Move to s₀ ;
-- 	Scan graph for changed edge costs;
-- 	if any edge costs changed
-- 		k m = k m + h(s₊ ,s₀);
-- 		s₊ = s₀;
-- 		for all directed edges (u,v) with changed edge costs
-- 			c old = c(u,v) ;
-- 			Update the edge costc ( u,v) ;
-- 			if (c old > c( u,v))
-- 				if ( u ≠ s goal)
-- 					rhs(u) = min(rhs(u), c(u,v) + g(v)) ;
-- 			else if (rhs(u)= c old + g(v))
-- 				if (u ≠ s goal)
-- 					rhs(u) = min s′∈Succ(u) (c(u,s′)+ g(s′)) ;
-- 			UpdateVertex (u) ;
-- 		ComputeShortestPath() ;


