-- procedure CalculateKey(s)
--  return [ min(g(s), rhs(s)) + h(s₀, s) + k m,
--       min(g(s), rhs(s)) ];

-- procedure Initialize()
--  U = ∅;
--  k m =0;
--  for all s∈S
--    rhs(s) = g(s) = ∞;
--  rhs(s goal) = 0;
--  U.Insert(s goal, [h(s₀ ,s goal); 0]);
-- 
-- procedure UpdateVertex(u)
--  if(    g(u) ≠ rhs ( u) AND u ∈ U) U.Update(u,CalculateKey(u));
--  else if( g(u) ≠ rhs ( u) AND u ∉ U) U.Insert(u,CalculateKey(u));
--  else if( g(u) = rhs ( u) AND u ∈ U) U.Remove(u);
-- 
-- procedure ComputeShortestPath()
--  while ( U.TopKey() < CalculateKey(s₀) OR rhs(s₀) > g(s₀))
--    u = U.Top() ;
--    k old = U.TopKey() ;
--    k new = CalculateKey(u)) ;
--    if ( k old < k new)
--      U.Update(u,k new);
--    else if(g(u) > rhs(u))
--      g(u)= rhs(u);
--      U.Remove(u) ;
--      for all s∈Pred(u)
--        if (s ≠ s goal)
--          rhs(s) = min( rhs(s), c(s,u) + g(u));
--        UpdateVertex ( s) ;
--    else
--      g old = g(u) ;
--      g(u)= ∞ ;
--      for all s∈Pred(u) ∪ {u}
--        if (rhs(s) = c (s,u) + g old)
--          if (s ≠ s goal)
--            rhs(s) = min s′∈Succ(s) (c(s,s′)+ g (s′));
--        UpdateVertex (s);
--
-- procedure Main ()
--  s₊ = s₀ ;
--  Initialize () ;
--  ComputeShortestPath () ;
--  while ( s₀ ≠ s goal)
--  /* if ( g ( s₀)= ∞) then there is no known path */
--  s₀ = argmin s ∈ Succ(s₀) (c(s₀ ,s′) + g(s′)) ;
--  Move to s₀ ;
--  Scan graph for changed edge costs;
--  if any edge costs changed
--    k m = k m + h(s₊ ,s₀);
--    s₊ = s₀;
--    for all directed edges (u,v) with changed edge costs
--      c old = c(u,v) ;
--      Update the edge costc ( u,v) ;
--      if (c old > c( u,v))
--        if ( u ≠ s goal)
--          rhs(u) = min(rhs(u), c(u,v) + g(v)) ;
--      else if (rhs(u)= c old + g(v))
--        if (u ≠ s goal)
--          rhs(u) = min s′∈Succ(u) (c(u,s′)+ g(s′)) ;
--      UpdateVertex (u) ;
--    ComputeShortestPath() ;

{-# LANGUAGE DeriveGeneric #-}
--module PathFinding (findPath) where

import GHC.Generics (Generic)
import Data.Hashable

import qualified Data.Vector.Unboxed as V
import qualified Data.Vector.Unboxed.Mutable as MV
import qualified Data.IntPSQ as ISQ
import Data.List (foldl')

import Control.Monad (when, foldM)
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

data Location = Location Int Int Int deriving (Read, Show, Eq, Ord, Generic)
data Goal = Goal Int Int deriving (Read, Show, Eq, Ord, Generic)
data Start = Start Int Int deriving (Read, Show, Eq, Ord, Generic)
instance Hashable Location

h :: Location -> Location -> Int
h (Location ax ay _) (Location bx by _) = max (abs (ax - bx)) (abs (ay - by))

vpos :: Int -> Int-> Int
vpos x y = y * yGridSize + x

neighbors (Location x y _) = map (\(dx,dy) -> Location dx dy (dy * yGridSize + dx)) $ neighborhood x y
  where
    neighborhood x y
      | xMin < x  && x < xMax  && yMin < y  && y < yMax  = [(x-1,y),(x+1,y),(x-1,y-1),(x-1,y+1),(x,y-1),(x,y+1),(x+1,y-1),(x+1,y+1)]
      | xMin >= x && x < xMax  && yMin < y  && y < yMax  = [(x+1,y),(x,y-1),(x,y+1),(x+1,y-1),(x+1,y+1)]
      | xMin < x  && x >= xMax && yMin < y  && y < yMax  = [(x-1,y-1),(x-1,y),(x-1,y+1),(x,y-1),(x,y+1)]
      | xMin < x  && x < xMax  && yMin >= y && y < yMax  = [(x-1,y),(x,y+1),(x+1,y),(x-1,y+1),(x+1,y+1)]
      | xMin < x  && x < xMax  && yMin < y  && y >= yMax = [(x-1,y-1),(x-1,y),(x,y-1),(x+1,y-1),(x+1,y)]
      | xMin >= x && x < xMax  && yMin >= y && y < yMax  = [(x,y+1),(x+1,y),(x+1,x+1)]     -- NW corner
      | xMin >= x && x < xMax  && yMin < y  && y >= yMax = [(x,y-1),(x+1,y-1),(x+1,y)]   -- SW corner
      | xMin < x  && x >= xMax && yMin >= y && y < yMax  = [(x-1,y),(x-1,y+1),(x,y+1)]   -- NE corner
      | xMin < x  && x >= xMax && yMin < y  && y >= yMax = [(x-1,y-1),(x-1,y),(x,y-1)] -- SE corner


type Cost       = Int
type DeltaCost    = Int
type Terrain    = V.Vector Cost
type EstimateCache  = V.Vector Cost
type ForwardCache   = V.Vector Cost

newtype Priority = Priority (Int, Int) deriving (Read, Show, Eq, Ord)
type PathHeap = ISQ.IntPSQ Priority Location

data Env = Env { terrain  :: Terrain
        , goal    :: Location
        , start   :: Location
        , dK :: Int}

type PathEnv = ReaderT Env
data PathVars = PathVars{ getEC :: EstimateCache, getFC :: ForwardCache, getPQ :: PathHeap }
type PathState = State PathVars


printVector :: Bool -> V.Vector Cost -> IO ()
printVector bool vector = putStrLn . concat $ V.ifoldr' print [] vector
  where
    print i x str 
      | i == 0 = ["_"] ++ ( map (\i -> "_"++ show i ++"_") . take xGridSize $ iterate (+1) 0) ++ ["\n\n"] ++ [show (quot i yGridSize) ++ " "] ++ p i x
      | rem i yGridSize == 0 = [show (quot i yGridSize) ++ " "] ++ p i x
      | otherwise = p i x
      where p i x = if rem (i+1) xGridSize == 0 then symbol x :"\n":str else symbol x : str

    symbol x = case compare x costMax of
      EQ -> "∞"
      GT -> "*"
      LT -> if bool then "." else " " ++ show (rem x 10) ++ " "

terrain1 :: Terrain
terrain1 = V.generate (xGridSize * yGridSize) (const 1)

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

initialize :: Goal -> Start -> Terrain -> (Env, PathVars)
initialize (Goal gx gy) (Start sx sy) terrain = (pathenv, pathvars)
  where
    gpos = vpos gx gy
    goal = Location gx gy gpos
    start = Location sx sy $ vpos sx sy

    estimate :: V.Vector Int
    estimate = V.replicate (xGridSize * yGridSize) costMax

    forward :: V.Vector Int
    forward = V.replicate (xGridSize * yGridSize) costMax V.// [(gpos, 0)]
    
    dK = 0

    pathenv  = Env terrain goal start dK
    pathvars = PathVars estimate forward $ ISQ.singleton gpos (Priority (h goal start, 0)) goal

test = do
  let (PathVars ec fc pq) = findPath (Goal 0 50) (Start 99 50) terrain
--  printVector False t
--  putStrLn "-----------------------------------"
  printVector True ec
  where 
    terrain = terrain1

-- procedure Main ()
--  s₊ = s₀ ;
--  Initialize () ;
--  ComputeShortestPath () ;
--  while ( s₀ ≠ s goal)
--  /* if ( g ( s₀)= ∞) then there is no known path */
--  s₀ = argmin s ∈ Succ(s₀) (c(s₀ ,s′) + g(s′)) ;
--  Move to s₀ ;
--  Scan graph for changed edge costs;
--  if any edge costs changed
--    k m = k m + h(s₊ ,s₀);
--    s₊ = s₀;
--    for all directed edges (u,v) with changed edge costs
--      c old = c(u,v) ;
--      Update the edge costc ( u,v) ;
--      if (c old > c( u,v))
--        if ( u ≠ s goal)
--          rhs(u) = min(rhs(u), c(u,v) + g(v)) ;
--      else if (rhs(u)= c old + g(v))
--        if (u ≠ s goal)
--          rhs(u) = min s′∈Succ(u) (c(u,s′)+ g(s′)) ;
--      UpdateVertex (u) ;
--    ComputeShortestPath() ;
findPath :: Goal -> Start -> Terrain -> PathVars
findPath goal@(Goal gx gy) start@(Start sx sy) terrain 
  | sx /= gx || sy /= gy = undefined
  | otherwise = pathvars
  where
    computePathVars = execState ( runReaderT computeShortestPath pathenv ) pathvars
    nextLocation = evalState ( runReaderT cheapestMove pathenv )
    (pathenv, pathvars) = initialize goal start terrain

cheapestMove :: PathEnv PathState (Maybe Location)
cheapestMove = do
  (Env _ _ start _) <- ask
  g_start <- g start

  if g_start >= costMax then return Nothing else
    fmap Just (foldM minOfNeighbors start (neighbors start))

  where minOfNeighbors s s' = do
    g_s  <- g s
    c_s  <- c s
    g_s' <- g s'
    c_s' <- c s'
    return $ if g_s + c_s <= g_s' + c_s' then s else s'

computeShortestPath :: PathEnv PathState EstimateCache
computeShortestPath = do
  (Env _ goal start _) <- ask

  prio_start    <- calcPrio start
  (top, prio_top) <- getTopU
  prio_top'     <- calcPrio top
  r_start     <- r start
  g_start     <- g start

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
      let n = neighbors top
      forM_ (filter (/= goal) n) (\s -> do
          r_s <- r s
          c_s <- c s
          rs s . min r_s $ c_s + g_top'
        )
      forM_ n updatePQ
      computeShortestPath
    else do
      gs top costMax
      let n = neighbors top
      forM_ (filter (/= goal) (top:n)) (\s -> do
        r_s <- r s
        c_s <- c s
        when (r_s == c_s + g_top) $ do
          r'_s <- foldl' minOfNeighbors (return costMax) (neighbors s)
          rs s r'_s
        )
      forM_ (top:n) updatePQ
      computeShortestPath
  else gets getEC
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
upsertPriority p s@(Location x y pos) = do
  (PathVars ec fc pq) <- get
  put . PathVars ec fc $ snd (ISQ.alter (const (p, Just (p, s))) pos pq)

removeKey :: Location -> PathEnv PathState ()
removeKey (Location _ _ pos) = do
  (PathVars ec fc pq) <- get
  put . PathVars ec fc $ ISQ.delete pos pq

getTopU :: PathEnv PathState (Location, Priority)
getTopU = do
  (Env _ _ start _) <- ask
  prio_start <- calcPrio start
  pq <- gets getPQ
  return $ case ISQ.findMin pq of
    Nothing -> (start, prio_start)
    Just (k,p,v)  -> (v,p)

updatePriority :: Location -> Priority -> PathEnv PathState ()
updatePriority s@(Location x y pos) p = do
  (PathVars ec fc pq) <- get
  put (PathVars ec fc $ snd (ISQ.alter (const (0, Just (p, s))) pos pq))

  
findMovementCost :: Location -> PathEnv PathState Cost
findMovementCost (Location _ _ pos) = do
  (Env costs _ _ _) <- ask
  return $ costs V.! pos
c = findMovementCost

findGValue :: Location -> PathEnv PathState Cost
findGValue (Location _ _ pos) = do
  (PathVars ec _ _) <- get
  return $ ec `V.unsafeIndex` pos
g = findGValue

findLookAheadValue :: Location -> PathEnv PathState Cost
findLookAheadValue (Location _ _ pos) = do
  (PathVars _ fc _) <- get
  return $ fc `V.unsafeIndex` pos
r = findLookAheadValue

setGValue :: Location -> Cost -> PathEnv PathState ()
setGValue s@(Location x y pos) val = do
  (PathVars ec fc pq) <- get
  let ec' = mutableWrite pos val ec
  put (PathVars ec' fc pq)
gs = setGValue


setLookAheadValue :: Location -> Cost -> PathEnv PathState ()
setLookAheadValue s@(Location x y pos) val = do
  (PathVars ec fc pq) <- get
  let fc' = mutableWrite pos val fc
  put (PathVars ec fc' pq)
rs = setLookAheadValue


mutableWrite i val v = runST (do
    mv <- V.unsafeThaw v
    MV.write mv i val
    V.unsafeFreeze mv)

calcPrio :: Location -> PathEnv PathState Priority
calcPrio s = do
  (Env c goal start dK) <- ask
  g_cost <- findGValue s
  r_cost <- findLookAheadValue s
  return $ calcPriority start dK g_cost r_cost s

calcPriority :: Location -> DeltaCost -> Cost -> Cost -> Location -> Priority
calcPriority start delta_cost estimate lookahead location =
  Priority ( m + h start location + delta_cost, m)
    where m = estimate `seq` lookahead `seq` min estimate lookahead



main = test
