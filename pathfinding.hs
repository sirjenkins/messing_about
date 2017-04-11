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
--
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

--module PathFinding () where

{-# LANGUAGE DeriveGeneric #-}
import GHC.Generics (Generic)
import Data.Hashable

import qualified Data.Vector.Unboxed as V
import qualified Data.Vector.Unboxed.Mutable as MV
import qualified Data.HashPSQ as HSQ
import Data.List (foldl')

import Control.Monad.State
import Control.Monad.ST (runST)
import Control.Monad.Reader

xGridSize = 100 :: Int
yGridSize = 100 :: Int
xMin = 0
xMax = xGridSize - 1
yMin = 0
yMax = yGridSize - 1

costMax = 1000000 :: Int

data Location = Location Int Int deriving (Read, Show, Eq, Ord, Generic)
data Goal = Goal Int Int deriving (Read, Show, Eq, Ord, Generic)
data Start = Start Int Int deriving (Read, Show, Eq, Ord, Generic)
instance Hashable Location

--instance Eq Location where
--	s == t = (destructure s) == (destructure t)
--		where
--			destructure u = case u of
--				(Goal x y) -> (x,y)
--				(Start x y) -> (x,y)
--				(Location x y) -> (x,y)


h :: Location -> Location -> Int
h (Location ax ay) (Location bx by) = max (abs (ax - bx)) (abs (ay - by))

vpos :: Location -> Int
vpos (Location x y) = y * yGridSize + x

neighbors s@(Location x y) = map (\(dx,dy) -> Location (x+dx) (y+dy)) $ neighborhood s
	where
		neighborhood (Location x y)
			| xMin < x  && x < xMax  && yMin < y  && y < yMax  = [(-1,-1),(-1,0),(-1,1),(0,-1),(0,1),(1,-1),(1,0),(1,1)]
			| xMin >= x && x < xMax  && yMin < y  && y < yMax  = [(0,-1),(0,1),(1,-1),(1,0),(1,1)]
			| xMin < x  && x >= xMax && yMin < y  && y < yMax  = [(-1,-1),(-1,0),(-1,1),(0,-1),(0,1)]
			| xMin < x  && x < xMax  && yMin >= y && y < yMax  = [(-1,0),(-1,1),(0,1),(1,0),(1,1)]
			| xMin < x  && x < xMax  && yMin < y  && y >= yMax = [(-1,-1),(-1,0),(0,-1),(1,-1),(1,0)]
			| xMin >= x && x < xMax  && yMin >= y && y < yMax  = [(0,1),(1,0),(1,1)]     -- NW corner
			| xMin >= x && x < xMax  && yMin < y  && y >= yMax = [(0,-1),(1,-1),(1,0)]   -- SW corner
			| xMin < x  && x >= xMax && yMin >= y && y < yMax  = [(-1,0),(-1,1),(0,1)]   -- NE corner
			| xMin < x  && x >= xMax && yMin < y  && y >= yMax = [(-1,-1),(-1,0),(0,-1)] -- SE corner


type Cost 			= Int
type DeltaCost 		= Int
type Terrain 		= V.Vector Cost
type EstimateCache 	= V.Vector Cost
type ForwardCache 	= V.Vector Cost

newtype Priority = Priority (Int, Int) deriving (Read, Show, Eq, Ord)
type PathHeap = HSQ.HashPSQ Location Priority Location

data Env = Env { terrain 	:: Terrain
						, goal 		:: Location
						, start 	:: Location
						, calcPriority'  :: Cost -> Cost -> Location -> Priority }

type PathEnv = ReaderT Env
data PathVars = PathVars{ getEC :: EstimateCache, getFC :: ForwardCache, getPQ :: PathHeap }
type PathState = State PathVars

test = do
	let (ec,t) = findPath (Goal 0 0) (Start 99 99)
	printVector False t
	putStrLn "-----------------------------------"
	printVector True ec

printVector :: Bool -> V.Vector Cost -> IO ()
printVector bool vector = putStrLn . concat $ V.ifoldr' print [] vector
	where
		print i x str = if i == 0
			then ["_"] ++ ( map (\i -> "_"++(show i)++"_") . take xGridSize $ iterate (\i -> i+1) 0) ++ ["\n\n"] ++ [(show (quot i yGridSize)) ++ " "] ++ (p i x)
			else if (rem i yGridSize) == 0 then [(show (quot i yGridSize)) ++ " "] ++ (p i x)
			else p i x

			where p i x = if (rem (i+1) xGridSize) == 0 then (symbol x):"\n":str else (symbol x):str

		symbol x = case compare x costMax of
			EQ -> "∞"
			GT -> "*"
			LT -> if bool then "." else " " ++ (show (rem x 10)) ++ " "

terrain1and5 :: Terrain
terrain1and5 = V.generate (xGridSize * yGridSize) (\i -> if mod i 3 == 0 || mod i 2 == 1 then 1 else 5)
--terrainNoPathCorner = V.generate (xGridSize * yGridSize) (\i -> if i == 88 || i == 98 || i == 89 then costMax else 5)
--terrainNoPathWall = V.generate (xGridSize * yGridSize) (\i -> if i >= 70 && i < 80 then costMax else 5)
--
--terrain138 :: Terrain
--terrain138 = V.generate (xGridSize * yGridSize) (\i -> if rem i 3 == 0 || rem i 2 == 1 || rem i 5 == 0 then 1 else 8)
--terrain325 :: Terrain
--terrain325 = V.generate (xGridSize * yGridSize) (\i -> if rem i 3 == 0 || rem i 2 == 1 || rem i 5 > 4 then 8 else 1)
--
--terrain4 :: Terrain
--terrain4 = V.generate (xGridSize * yGridSize) (\i -> if rem i 4 <= 1 then 1 else 8)

findPath (Goal gx gy) (Start sx sy) = (evalState ( runReaderT computeShortestPath (Env terrain goal start calcPrio) ) pathvars, terrain)
	where
		goal = Location gx gy
		start = Location sx sy

		terrain = terrain1and5
		estimate :: V.Vector Int
		estimate = V.replicate (xGridSize * yGridSize) costMax

		forward :: V.Vector Int
		forward = V.replicate (xGridSize * yGridSize) costMax V.// [(vpos goal, 0)]
		
		dK = 0

		calcPrio :: Cost -> Cost -> Location -> Priority
		calcPrio = calcPriority start dK h

		pathvars = PathVars estimate forward $ HSQ.singleton goal (Priority (h goal start, 0)) goal

computeShortestPath :: PathEnv PathState EstimateCache
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
			g_top' <- g top
			removeKey top
			forM_ (filter (\x -> x /= goal) (neighbors top)) (\s -> do
					r_s <- r s
					c_s <- c s
					rs s . min r_s $ c_s + g_top'
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
	else gets getEC >>= (\ec -> return ec)
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
	let x = ec
	put . PathVars ec fc $ snd (HSQ.alter (\_ -> (0,Just (p, s))) s pq)

removeKey :: Location -> PathEnv PathState ()
removeKey s = do
	(PathVars ec fc pq) <- get
	put . PathVars ec fc $ HSQ.delete s pq

getTopU :: PathEnv PathState (Location, Priority)
getTopU = do
	start <- asks start
	prio_start <- calcPrio start
	pq <- gets getPQ
	return $ case HSQ.findMin pq of
		Nothing -> (start, prio_start)
		Just (k,p,v)  -> (k,p)

updatePriority :: Location -> Priority -> PathEnv PathState ()
updatePriority s p = do
	(PathVars ec fc pq) <- get
	put (PathVars ec fc $ snd (HSQ.alter (\_ -> (0, Just (p, s))) s pq))

	
findMovementCost :: Location -> PathEnv PathState Cost
findMovementCost s = do
	costs <- asks terrain
	return $ costs V.! (vpos s)
c = findMovementCost

findGValue :: Location -> PathEnv PathState Cost
findGValue s = do
	(PathVars ec fc pq) <- get
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
--	let ec' = ec V.// [(vpos s, val)]
	let ec' = mutableWrite (vpos s) val ec
	put (PathVars ec' fc pq)
gs = setGValue


setLookAheadValue :: Location -> Cost -> PathEnv PathState ()
setLookAheadValue s val = do
	(PathVars ec fc pq) <- get
	let fc' = mutableWrite (vpos s) val fc
	put (PathVars ec fc' pq)
rs = setLookAheadValue


mutableWrite i val v = runST (do
		mv <- V.unsafeThaw v
		MV.write mv i val
		V.unsafeFreeze mv)

calcPrio :: Location -> PathEnv PathState Priority
calcPrio s = do
	(Env c goal start cP) <- ask
	g_cost <- findGValue s
	r_cost <- findLookAheadValue s
	return ( cP g_cost r_cost s )

calcPriority :: Location -> DeltaCost -> (Location -> Location -> Cost) -> Cost -> Cost -> Location -> Priority
calcPriority start delta_cost heuristic estimate lookahead location =
	Priority ( (min estimate lookahead) + (heuristic start location) + delta_cost, min estimate lookahead )



main = test
