module Terrain (
    xGridSize
    , yGridSize
    , xMin
    , xMax
    , yMin
    , yMax
    , costMax
  ) where


xGridSize = 100 :: Int
yGridSize = 100 :: Int
xMin = 0 :: Int
xMax = xGridSize - 1 :: Int
yMin = 0 :: Int
yMax = yGridSize - 1 :: Int

costMax = 1000000 :: Int
