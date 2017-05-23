module Terrain (
    xGridSize
    , yGridSize
    , xMin
    , xMax
    , yMin
    , yMax
    , costMax
    , (!)
    , convert
    , generateTerrain
    , mkLocation
    , neighbors
    , Location
    , Terrain
    , TerrainChange (TerrainChange)
  ) where

import qualified Data.Vector.Unboxed as V


xGridSize = 10 :: Int
yGridSize = 10 :: Int
xMin = 0 :: Int
xMax = xGridSize - 1 :: Int
yMin = 0 :: Int
yMax = yGridSize - 1 :: Int

costMax = 1000000 :: Int

data Location = Location !Int !Int deriving (Read, Show, Eq, Ord)
newtype Terrain = Terrain (V.Vector Int) deriving (Read)
data TerrainChange  = TerrainChange Location Int

instance Show Terrain where
  show (Terrain vector) = (concat $ V.ifoldr' print [] vector) ++ "\n"
    where
      print i x str
        | rem i yGridSize == 0 = (show (quot i yGridSize) ++ " ") : p i x
        | otherwise = p i x
        where p i x = if rem (i+1) xGridSize == 0 then symbol x :"\n":str else symbol x : str

      symbol x = case compare x costMax of
        EQ -> "∞"
        GT -> "*"
        LT -> "."


mkLocation :: Int -> Int -> Location
mkLocation x y = Location x y

convert :: (Int -> Int -> a) -> Location -> a
convert f (Location x y) = f x y

vpos :: Location -> Int
vpos (Location x y) = y * yGridSize + x

(!) :: Terrain -> Location -> Int
(!) (Terrain v) location = v V.! vpos location

update :: Location -> Int -> Terrain
update = undefined

neighbors :: ((Int, Int) -> b) -> Location -> [b]
neighbors f (Location x y) = map f $ neighborhood x y
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


generateTerrain :: (Int -> Int -> Int) -> Terrain
generateTerrain g = Terrain $ V.generate length f
  where
    length = xGridSize * yGridSize
    f v_index = g (v_index `rem` xGridSize) (v_index `quot` yGridSize)

terrain1 :: Terrain
terrain1 = Terrain $ V.generate (xGridSize * yGridSize) (const 1)

terrainSplitVertical :: Terrain
terrainSplitVertical = Terrain $ V.generate (xGridSize * yGridSize) (\i -> if odd (i `div` 50) && i `rem` 50 == 0 then costMax else 1)

