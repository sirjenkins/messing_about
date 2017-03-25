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

xMax = 10 :: Int
yMax = 10 :: Int
costMax = 1000 :: Int

data Location = Location Int Int deriving (Read, Show, Eq, Ord)
data Goal 	  = Goal Int Int 	 deriving (Read, Show, Eq, Ord)
data Start 	  = Start Int Int 	 deriving (Read, Show, Eq, Ord)

class Path a where
	h :: Path b => a -> b -> Int
	y :: a -> Int
	x :: a -> Int

instance Path Location where
	h a b = 0
	y (Location x y) = y
	x (Location x y) = x

instance Path Goal where
	h a b = 0
	y (Goal x y) = y
	x (Goal x y) = x

instance Path Start where
	h a b = 0
	y (Start x y) = y
	x (Start x y) = x


type Cost 			= Int
type DeltaCost 		= Int
type Terrain 		= V.Vector Cost
type EstimateCost 	= V.Vector Cost
type ForwardCost 	= V.Vector Cost

newtype Priority = Priority (Int, Int) deriving (Read, Show, Eq, Ord)
type PathHeap = U.PSQ Location Priority

data PathEnv = PathEnv { terrain 	:: Terrain
						, goal 		:: Goal
						, start 	:: Start
						, deltaK 	:: Cost }

data PathVars = PathVars{ getEstimate :: EstimateCost, getForward :: ForwardCost }
type PathState = State PathVars

vpos :: Path a => a -> Int
vpos s = (y s) * yMax + (x s)

computeShortestPath :: ReaderT PathEnv PathState EstimateCost
computeShortestPath = do
	env <- ask
	ss  <- get
	
	let s0 = start env
	let s0_prio = calcPrio s0 ss env

	return $ getEstimate ss
	where
		calcPrio s ss env = evalState ( runReaderT (calcPriority s) env) ss

calcPriority :: Path a => a -> ReaderT PathEnv PathState Priority
calcPriority s = do
	(PathVars e f) <- get
	let (g, rhs) = (e V.! vpos s, f V.! vpos s)
	dK <- asks deltaK
	s0 <- asks start
	
	return $ Priority ( (min g rhs) + (h s0 s) + dK, min g rhs )

hook = evalState ( runReaderT computeShortestPath (PathEnv terrain (Goal 9 9) (Start 0 0) 0) ) pathvars
	where
		terrain :: V.Vector Int
		terrain = V.generate (xMax * yMax) (\i -> if mod i 3 == 0 || mod i 2 == 1 then 1 else 5)

		estimate :: V.Vector Int
		estimate = V.replicate (xMax * yMax) costMax

		forward :: V.Vector Int
		forward = V.replicate (xMax * yMax) costMax

		pathvars = PathVars estimate forward


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


