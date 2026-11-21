"""Create a USD variant with the base-link COM shifted, to test whether a
ballast change makes the rear-right wheel liftable in right translation."""
import sys
from pxr import Usd, UsdPhysics, Gf

src, dst = sys.argv[1], sys.argv[2]
dy = float(sys.argv[3]) if len(sys.argv) > 3 else 0.015
dx = float(sys.argv[4]) if len(sys.argv) > 4 else 0.0

stage = Usd.Stage.Open(src)
base = stage.GetPrimAtPath("/AS2W/Geometry/base_link")
if not base:
    raise SystemExit("base_link not found")
m = UsdPhysics.MassAPI(base)
com = m.GetCenterOfMassAttr().Get()
print("original com", com)
new = Gf.Vec3f(com[0] + dx, com[1] + dy, com[2])
m.CreateCenterOfMassAttr().Set(new)
print("new com", new)
stage.Export(dst)
print("wrote", dst)
