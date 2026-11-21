from pxr import Usd, UsdPhysics

SRC = "/root/as2w/unitree-flat-v2-tanh/generated-v2/as2w-t2-v2/as2w-t2-v2.usd"
stage = Usd.Stage.Open(SRC)
prims = [p for p in stage.Traverse() if "base_link" in p.GetName()]
print("base prims:", [str(p.GetPath()) for p in prims][:6])
for p in prims[:6]:
    m = UsdPhysics.MassAPI(p)
    print(str(p.GetPath()), "rigid=", p.HasAPI(UsdPhysics.RigidBodyAPI),
          "mass_api=", p.HasAPI(UsdPhysics.MassAPI),
          "mass=", m.GetMassAttr().Get(), "com=", m.GetCenterOfMassAttr().Get())
