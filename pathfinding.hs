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

import Data.List (foldl')
import qualified Data.Vector.Unboxed as V
import qualified Data.PSQueue as PSQ
import Control.Monad.State
import Control.Monad.Reader

xMax = 10 :: Int
yMax = 10 :: Int
costMax = 1000 :: Int

data Location = Location Int Int | Goal Int Int | Start Int Int deriving (Read, Show, Ord)
instance Eq Location where
	s == t = (destructure s) == (destructure t)
		where
			destructure u = case u of
				(Goal x y) -> (x,y)
				(Start x y) -> (x,y)
				(Location x y) -> (x,y)


h :: Location -> Location -> Int
h a b = let
	(ax, ay) = destructure a
	(bx, by) = destructure b
	in max (abs (ax - bx)) (abs (ay - by))
	where
		destructure s = case s of
			(Goal x y) -> (x,y)
			(Start x y) -> (x,y)
			(Location x y) -> (x,y)


coord :: Location -> (Int, Int)
coord (Goal x y) = (x,y)
coord (Start x y) = (x,y)
coord (Location x y) = (x,y)

neighbors (Goal x y) = neighbors (Location x y)
neighbors (Start x y) = neighbors (Location x y)
neighbors (Location x y) = 
	map (\(x,y) -> Location x y) . filter withinBounds $ map (\(dx,dy) -> (x+dx, y+dy)) neighborhood
	where
		withinBounds (x,y) = x >= 0 && x < xMax && y >= 0 && y < yMax
		neighborhood = [(-1,-1),(-1,0),(-1,1),(0,-1),(0,1),(1,-1),(1,0),(1,1)]


type Cost 			= Int
type DeltaCost 		= Int
type Terrain 		= V.Vector Cost
type EstimateCache 	= V.Vector Cost
type ForwardCache 	= V.Vector Cost

newtype Priority = Priority (Int, Int) deriving (Read, Show, Eq, Ord)
type PathHeap = PSQ.PSQ Location Priority

data Env = Env { terrain 	:: Terrain
						, goal 		:: Location
						, start 	:: Location
						, calcPriority'  :: Cost -> Cost -> Location -> Priority }

type PathEnv = ReaderT Env
data PathVars = PathVars{ getEC :: EstimateCache, getFC :: ForwardCache, getPQ :: PathHeap }
type PathState = State PathVars

vpos :: Location -> Int
vpos s = y * yMax + x
	where (x,y) = coord s

computeShortestPath :: PathEnv PathState ()
computeShortestPath = do
	start <- asks start
	goal <- asks goal

	prio_start 		<- calcPrio start
	(top, prio_top) <- getTopU
	prio_top' 		<- calcPrio top
	r_start 		<- r start
	g_start 		<- g start

	r_top <- r top
	g_top <- g top

	if prio_top < prio_start || r_start > g_start then
		if prio_top < prio_top' then do
			updatePriority top prio_top'
			computeShortestPath
		else if g_top > r_top then do
			gs top r_top
			removeKey top
			forM_ (filter (\x -> x /= goal) (neighbors top)) (\s -> do
					r_s <- r s
					c_s <- c s
					rs s . min r_s $ c_s + g_top
				)
			forM_ (neighbors top) (\s -> updatePQ s)
			computeShortestPath
		else do
			gs top costMax
			forM_ (filter (\x -> x /= goal) (top:(neighbors top))) (\s -> do
				r_s <- r s
				c_s <- c s
				if r_s == c_s + g_top then do
					r'_s <- foldl' minOfNeighbors (return costMax) $ (neighbors s)
					rs s r'_s
				else return ()
				)
			forM_ (top:(neighbors top)) (\s -> updatePQ s)
			computeShortestPath
	else return ()
	where
		minOfNeighbors s s' = do
			r_s <- s
			g_s' <- g s'
			c_s' <- c s'
			return $ min r_s (g_s' + c_s')

updatePQ :: Location -> PathEnv PathState ()
updatePQ u = do
	g_u <- g u
	r_u <- r u
	prio_u <- calcPrio u
	if g_u /= r_u then
		upsertPriority prio_u u
	else
		removeKey u

upsertPriority :: Priority -> Location -> PathEnv PathState ()
upsertPriority p s = do
	(PathVars ec fc pq) <- get
	put . PathVars ec fc $ PSQ.alter (\_ -> Just p) s pq

removeKey :: Location -> PathEnv PathState ()
removeKey s = do
	(PathVars ec fc pq) <- get
	put . PathVars ec fc $ PSQ.delete s pq

getTopU :: PathEnv PathState (Location, Priority)
getTopU = do
	start <- asks start
	prio_start <- calcPrio start
	pq <- gets getPQ
	return $ case PSQ.findMin pq of
		Nothing -> (start, prio_start)
		Just b  -> (PSQ.key b, PSQ.prio b)

updatePriority :: Location -> Priority -> PathEnv PathState ()
updatePriority s p = do
	(PathVars ec fc pq) <- get
	put (PathVars ec fc $ PSQ.update (\_ -> Just p) s pq)

	
findMovementCost :: Location -> PathEnv PathState Cost
findMovementCost s = do
	costs <- asks terrain
	return $ costs V.! (vpos s)
c = findMovementCost

findGValue :: Location -> PathEnv PathState Cost
findGValue s = do
	ec <- gets getEC
	return $ ec V.! (vpos s)
g = findGValue

findLookAheadValue :: Location -> PathEnv PathState Cost
findLookAheadValue s = do
	(PathVars ec fc pq) <- get
	return $ fc V.! (vpos s)
r = findLookAheadValue

setGValue :: Location -> Cost -> PathEnv PathState ()
setGValue s val = do
	(PathVars ec fc pq) <- get
	let ec' = ec V.// [(vpos s, val)]
	put (PathVars ec' fc pq)
gs = setGValue

setLookAheadValue :: Location -> Cost -> PathEnv PathState ()
setLookAheadValue s val = do
	(PathVars ec fc pq) <- get
	let fc' = fc V.// [(vpos s, val)]
	put (PathVars ec fc' pq)
rs = setLookAheadValue

calcPrio :: Location -> PathEnv PathState Priority
calcPrio s = do
	calcPrio <- asks calcPriority'
	g_cost <- findGValue s
	r_cost <- findLookAheadValue s
	return ( calcPrio g_cost r_cost s )

calcPriority :: Location -> DeltaCost -> (Location -> Cost) -> Cost -> Cost -> Location -> Priority
calcPriority start delta_cost heuristic estimate lookahead location =
	Priority ( (min estimate lookahead) + (heuristic location) + delta_cost, min estimate lookahead )

init goal start = evalState ( runReaderT computeShortestPath (Env terrain goal start calcPrio) ) pathvars
	where
		terrain :: V.Vector Int
		terrain = V.generate (xMax * yMax) (\i -> if mod i 3 == 0 || mod i 2 == 1 then 1 else 5)

		estimate :: V.Vector Int
		estimate = V.replicate (xMax * yMax) costMax

		forward :: V.Vector Int
		forward = V.replicate (xMax * yMax) costMax
		
		dK = 0

		calcPrio :: Cost -> Cost -> Location -> Priority
		calcPrio = calcPriority start dK (h start)

		pathvars = PathVars estimate forward $ PSQ.singleton goal (Priority (h goal start, 0))


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


