{-# LANGUAGE FlexibleContexts #-}
import qualified Data.Vector.Unboxed as V
import qualified Data.Vector.Unboxed.Mutable as MV
import Control.Monad.State
import Control.Monad.ST (runST)
import Control.Monad.Reader
import Control.Monad.Primitive

ini = (V.singleton (1::Int), V.singleton (2::Int))

test (vo, ve) = runST $ runReaderT (evalStateT (inner vo ve) (V.singleton (1::Int))) 2

inner vo ve = do
  x <- get
  y <- ask
  mvo <- V.thaw vo
  mve <- V.thaw ve

  x <- MV.read mvo 0
  muteWrite mvo 0 x

  y <- MV.read mve 0
  muteWrite mve 0 y

  vo' <- freeze mvo
  ve' <- freeze mve
  put vo'
  return (vo', ve')

freeze :: (Control.Monad.Primitive.PrimMonad m) => MV.MVector (Control.Monad.Primitive.PrimState m) Int -> m (V.Vector Int)
freeze mv = V.unsafeFreeze mv

muteWrite v i val = MV.write v i $ val + 1
