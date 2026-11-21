"""Create a USD variant with extra base-link mass, to see how much of the
sim-to-real hip-torque gap is unmodelled payload."""
import sys

from pxr import Gf, Usd, UsdPhysics

src, dst = sys.argv[1], sys.argv[2]
add_kg = float(sys.argv[3])

stage = Usd.Stage.Open(src)
base = stage.GetPrimAtPath("/AS2W/Geometry/base_link")
if not base:
    raise SystemExit("base_link not found")
m = UsdPhysics.MassAPI(base)
mass = m.GetMassAttr().Get()
print("original mass", mass, "kg")
new = float(mass) + add_kg
m.CreateMassAttr().Set(new)
print("new mass", new, "kg")
stage.Export(dst)
print("wrote", dst)
