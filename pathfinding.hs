
{-# LANGUAGE DeriveGeneric #-}
--module PathFinding (newPath, adjustPath, PathContext) where

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
h (Location ax ay _) (Location bx by _) = 10 * (dx + dy) + (14 - 2 * 10) * (min dx dy)
  where
    dx = abs $ ax - bx
    dy = abs $ ay - by

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


type Cost           = Int
type DeltaCost      = Int
type Terrain        = V.Vector Cost
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

data PathContext = PathContext{ getEnv :: Env, getPathVars :: PathVars} | Empty
data TerrainChange = TerrainChange Location Cost
data CostChange = CostChange Location Cost

printVector :: Bool -> V.Vector Cost -> IO ()
printVector bool vector = putStrLn . concat $ V.ifoldr' print [] vector
  where
    print i x str
      | rem i yGridSize == 0 = [show (quot i yGridSize) ++ " "] ++ p i x
      | otherwise = p i x
      where p i x = if rem (i+1) xGridSize == 0 then symbol x :"\n":str else symbol x : str

    symbol x = case compare x costMax of
      EQ -> "∞"
      GT -> "*"
      LT -> if bool then "." else " " ++ show (rem x 10) ++ " "

terrain1 :: Terrain
terrain1 = V.generate (xGridSize * yGridSize) (const 1)

terrainSplitVertical :: Terrain
terrainSplitVertical = V.generate (xGridSize * yGridSize) (\i -> if odd (i `div` 50) && i `rem` 50 == 0 then costMax else 1)

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
--  printVector True terrain
  print s'
  printVector True ec
  where
--   terrain = terrainSplitVertical
    terrain = terrain1
--    start = Start 99 50
--    goal  = Goal 0 50
    start = Start 0 0
    goal  = Goal 99 99

    s0 = Location 50 50 $ vpos 99 50
    (s', ec) = case newPath goal start terrain of
      (Just loc, Just pc) -> (loc, getEC $ getPathVars pc)
      (Nothing, Just pc) -> (s0, getEC $ getPathVars pc)

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

newPath :: Goal -> Start -> Terrain -> (Maybe Location, Maybe PathContext)
newPath goal start terrain =
  let (pathenv, pathvars) = initialize goal start terrain
  in findPath pathenv pathvars []


updateContext :: Location -> PathContext -> [CostChange] -> PathContext
updateContext start' (PathContext pathenv@(Env terrain goal start dK) pathvars) changeList =
  let pathenv' = Env terrain goal start' $ dK + h start start'
      update = mapM_ updateVars changeList

  in PathContext pathenv' $ execState ( runReaderT update pathenv ) pathvars

updateVars :: CostChange -> PathEnv PathState ()
updateVars (CostChange v gridCost) = do
  (Env _ goal _ _) <- ask

  forM_ (filter (/= goal) (neighbors v)) (\u -> do
      r_u <- r u
      g_v <- g v
      c_v <- c u v
      let c_old = gridCost + moveCost u v

      if c_old > c_v then
        rs u . min r_u $ c_v + g_v
      else when (r_u == c_old + g_v) $
        foldl' (minCost v) (return costMax) (neighbors u) >>= (\r'_u -> rs u r'_u)

      updatePQ u
    )
  where
    minCost s0 sa sb = do
      r_sa <- sa
      g_sb <- g sb
      c_sb <- c s0 sb
      return $ min r_sa (g_sb + c_sb)

findPath :: Env -> PathVars -> [TerrainChange] -> (Maybe Location, Maybe PathContext)
findPath pathenv@(Env terrain goal start dK) pathvars changeList
  | goal == start = (Nothing, Nothing)
  | null changeList = (nextLocation, Just $ PathContext pathenv pathvars)
  | otherwise = (nextLocation, Just $ PathContext pathenv pathvars')
  where
    pathvars' = execState ( runReaderT computeShortestPath pathenv ) pathvars
    nextLocation = evalState ( runReaderT cheapestMove pathenv ) pathvars'

cheapestMove :: PathEnv PathState (Maybe Location)
cheapestMove = do
  (Env _ _ start _) <- ask
  let ns = neighbors start
  g_start <- g start

  s_next <- foldM (minOfNeighbors start) start $ neighbors start
  g_next <- g s_next
  return $ if g_next >= costMax then Nothing else Just s_next

  where
    minOfNeighbors s0 sa sb = do
      g_sa  <- g sa
      c_sa  <- c s0 sa
      g_sb <- g sb
      c_sb <- c s0 sb
      return $ if g_sa + c_sa <= g_sb + c_sb then sa else sb

computeShortestPath :: PathEnv PathState ()
computeShortestPath = do
  (Env _ goal start _) <- ask

  prio_start      <- calcPrio start
  (top, prio_top) <- getTopU
  prio_top'       <- calcPrio top
  r_start         <- r start
  g_start         <- g start

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
          c_s <- c top s
          rs s . min r_s $ c_s + g_top'
        )
      forM_ n updatePQ
      computeShortestPath
    else do
      gs top costMax
      let n = neighbors top
      forM_ (filter (/= goal) (top:n)) (\s -> do
        r_s <- r s
        c_s <- c top s
        when (r_s == c_s + g_top) $ do
          r'_s <- foldl' (minCost top) (return costMax) (neighbors s)
          rs s r'_s
        )
      forM_ (top:n) updatePQ
      computeShortestPath
  else return ()
  where
    minCost s0 sa sb = do
      r_sa <- sa
      g_sb <- g sb
      c_sb <- c s0 sb
      return $ min r_sa (g_sb + c_sb)

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

terrainCost :: Location -> Terrain -> Cost
terrainCost (Location _ _ pos) terrain = terrain V.! pos

moveCost :: Location -> Location -> Cost
moveCost (Location x y _) (Location x' y' _)
  | x == x' || y == y'  = 5
  | otherwise           = 7

findMovementCost :: Location -> Location -> PathEnv PathState Cost
findMovementCost s@(Location x y _) s'@(Location x' y' pos')
  | x == x' && y == y' = return 0
  | otherwise = ask >>= (\(Env costs _ _ _) -> return $ costs V.! pos' + moveCost s s')
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

