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
import Control.Monad.Reader
import Data.Maybe (fromMaybe)

xMax = 10 :: Int
yMax = 10 :: Int
costMax = 1000 :: Int

data Location
	= Location Int Int
	| Goal Int Int 
	| Start Int Int deriving (Read, Show, Eq, Ord)

x (Location x y) = x
x (Goal x y)	 = x
x (Start x y)	 = x
y (Location x y) = y
y (Goal x y) 	 = y
y (Start x y)	 = y

type Cost 			= Int
type DeltaCost 		= Int
type CostCache 		= V.Vector Cost
type EstimateCache 	= V.Vector Cost
type ForwardCache 	= V.Vector Cost

newtype Priority = Priority (Int, Int) deriving (Read, Show, Eq, Ord)
type PathHeap = U.PSQ Location Priority

data PathEnv = PathEnv {  cost 		:: CostCache
						, goal 		:: Location
						, start 	:: Location
						, deltaK 	:: Cost }

--data PathVars = PathVars {
--	  gset		:: Location -> Cost -> (Location -> Cost)
--	, g 		:: Location -> Cost
--	, rhsset 	:: Location -> Cost -> (Location -> Cost)
--	, rhs 		:: Location -> Cost
--	, calcPrio 	:: Int -> Int -> Location -> Priority
--	, getPQ 	:: PathHeap  }

--type PathState = State PathVars

-- need bounds checking
vpos :: Location -> Int
vpos s = (y s) * yMax + (x s)


neighbors (Goal x y) = neighbors (Location x y)
neighbors (Start  x y) = neighbors (Location x y)
neighbors s@(Location x y) = calcNeighbors neighborhood s
	where
		neighborhood :: [(Int, Int)]
		neighborhood = [(1,1),(1,0),(1,-1),(0,1),(0,-1),(-1,1),(-1,0),(-1,-1)]

		calcNeighbors :: [(Int, Int)] -> Location -> [Location]
		calcNeighbors [] s@(Location x y) = []
		calcNeighbors n (Goal x y) = calcNeighbors n (Location x y)
		calcNeighbors n (Start x y) = calcNeighbors n (Location x y)
		calcNeighbors ((dx, dy):ns) s@(Location x y)
			| x' >= xMax || x' < 0 = calcNeighbors ns s
			| y' >= yMax || y' < 0 = calcNeighbors ns s
			| otherwise = (Location x' y') : calcNeighbors ns s
			where
				x' = x + dx :: Int
				y' = y + dy :: Int

--      goal        start       -------------prio -------------------                       gs                               g                               rs                                r                 pq
loop :: Location -> Location -> (Cost -> Cost -> Location -> Priority) -> (Location -> Cost -> (Location -> Cost)) -> (Location -> Cost) -> (Location -> Cost -> (Location -> Cost)) -> (Location -> Cost) -> PathHeap -> (Location -> Cost)
loop goal start calcPrio gs g rs r pq
	| prio_top < prio_start || r start > g start =
		if prio_top < prio_top' then loop goal start calcPrio gs g rs r $ U.insert top prio_top' pq'
		else if g top > r top then undefined
-- 			g(u)= rhs(u);
-- 			U.Remove(u) ;
-- 			for all s∈Pred(u)
-- 				if (s ≠ s goal)
-- 					rhs(s) = min( rhs(s), c(s,u) + g(u));
-- 				UpdateVertex ( s) ;

		else undefined
	| otherwise = undefined
	where
		prio_start = calcPrio (g start) (r start) start
		(top, prio_top, pq') = case U.minView pq of
									Nothing -> (start, prio_start, pq)
									Just (b, pq') -> (U.key b, U.prio b, pq')
		prio_top' = calcPrio (g top) (r top) top

adjustForwards g_u r pq [] = (r, pq)
adjustForwards g_u r pq (s:ss) = undefined

init :: Location -> Location -> (Location -> Cost)
init goal start = 
	loop goal start calcPriority' gset' g' rhsset' rhs' pq
--		( calcPriority' (g' goal) (rhs' goal) goal )
--		( calcPriority' (g' start) (rhs' start) start )
--		( g' start )
--		( rhs' start )

	where

		initdK = 0
		initEstimate = V.replicate (xMax * yMax) costMax
		initForward = V.replicate (xMax * yMax) costMax

		gset' 	= setEstimateCache initEstimate
		g' 		= gset' goal costMax
		rhsset' = setForwardCache initForward
		rhs' 	= rhsset' goal 0
		pq 		= U.singleton goal $ calcPriority' (g' goal) (rhs' goal) goal
		calcPriority' = calcPriority initdK (h start)


--		pathvars = PathVars
--			gset' g'
--			rhsset' rhs'
--			calcPriority' . U.singleton goal $ calcPriority' (g' goal) (rhs' goal) goal

h a b = 0

terrain :: V.Vector Int
terrain = V.generate (xMax * yMax) (\i -> if mod i 3 == 0 || mod i 2 == 1 then 1 else 5)


estimateCache :: EstimateCache -> (Location -> Cost)
estimateCache estimate_cache = (V.!) estimate_cache . vpos

setEstimateCache :: EstimateCache -> (Location -> Cost -> (Location -> Cost))
setEstimateCache estimate_cache s c = estimateCache $ (V.//) estimate_cache [(vpos s, c)]

forwardCache :: ForwardCache -> (Location -> Cost)
forwardCache forward_cache = (V.!) forward_cache . vpos

setForwardCache :: ForwardCache -> (Location -> Cost -> (Location -> Cost))
setForwardCache forward_cache s c = forwardCache $ (V.//) forward_cache [(vpos s, c)]

calcPriority :: Int -> (Location -> Int) -> Cost -> Cost -> Location -> Priority
calcPriority dK h g rhs s = Priority ( (min g rhs) + (h s) + dK, min g rhs)

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


