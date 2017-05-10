module PathfindingSpec (spec) where

import Test.Hspec
import Data.Maybe (isJust, fromJust)
import Lib


plainTerrain = generateTerrain

spec = context "finding paths" $ do
  it "finds a path when goal and start are the same" $ do
    let goal = Goal 0 0
        start = Start 0 0
        (loc, path) = newPath goal start plainTerrain

    isJust loc `shouldBe` True
    fromJust loc `shouldBe` Location 0 0 0
