module PathfindingSpec (spec) where

import Debug.Trace
import Test.Hspec
import Data.Maybe (isJust, fromJust)
import Lib



spec = context "Basic Paths - flat plane, no terrain changes" $ do

  let plain = generateTerrain (const . const 1)
  let plain1 = generateTerrain (const . const 1)

  it "finds a path when goal (SE) and start (NW)" $ do
    let g = Goal xMax yMax
        s = Start xMin  yMin
        (x, p) = newPath g s plain1
--        (loc', path') = nextLocation start [] path

    traceM $ "g " ++ show g
    traceM $ "s " ++ show s
    traceM $ "HLJJFDLSFSJLSDJL " ++ show x
    traceM $ "Path " ++ show p
    isJust x `shouldBe` True
    fromJust x `shouldBe` mkLocation (xMin + 1) (yMin + 1)

  it "finds a path when goal (NW) and start (SE)" $ do
    let goal = Goal xMin yMin
        start = Start xMax yMax
        (loc, path) = newPath goal start plain
--        (loc', path') = nextLocation start [] path

    traceM $ "goal " ++ show goal
    traceM $ "start " ++ show start
    traceM $ "First test " ++ show loc
    traceM $ "First Path " ++ show path

    isJust loc `shouldBe` True
    fromJust loc `shouldBe` mkLocation (xMax - 1) (yMax - 1)
--
--    isJust loc' `shouldBe` True
--    fromJust loc' `shouldBe` mkLocation (xMax - 1) (yMax - 1)




  it "finds a path when goal (NE) and start (SW)" $ do
    let goal = Goal xMax yMin
        start = Start xMin yMax
        (loc, path) = newPath goal start plain
        (loc', path') = nextLocation start [] path

    isJust loc `shouldBe` True
    fromJust loc `shouldBe` mkLocation (xMin + 1) (yMax - 1)
--
--    isJust loc' `shouldBe` True
--    fromJust loc' `shouldBe` mkLocation (xMin + 1) (yMax - 1)
--
--
----    isJust loc' `shouldBe` True
----    fromJust loc' `shouldBe` mkLocation (xMin + 1) (yMin + 1)
----
  it "finds a path when goal (SW) and start (NE)" $ do
    let goal = Goal xMin yMax
        start = Start xMax yMin
        (loc, path) = newPath goal start plain
        (loc', path') = nextLocation start [] path

    isJust loc `shouldBe` True
    fromJust loc `shouldBe` mkLocation (xMax - 1) (yMin + 1)
----
----    isJust loc' `shouldBe` True
----    fromJust loc' `shouldBe` mkLocation (xMax - 1) (yMin + 1)
----
--  it "finds a paths when goal and start are next to each other" $ do
--    pending
--
--  it "finds a path when goal and start are the same" $ do
--    let goal = Goal xMin yMin
--        start = Start xMin yMin
--        (loc, path) = newPath goal start plain
--        (loc', path') = nextLocation start [] path
--
--    isJust loc `shouldBe` True
--    fromJust loc `shouldBe` mkLocation xMin yMin
--
--    isJust loc' `shouldBe` True
--    fromJust loc' `shouldBe` mkLocation xMin yMin


