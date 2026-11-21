from pxr import Usd, UsdPhysics, UsdShade

SRC = "/root/as2w/unitree-flat-v2-tanh/generated-v2/as2w-t2-v2/as2w-t2-v2.usd"
stage = Usd.Stage.Open(SRC)
seen = {}
for prim in stage.Traverse():
    path = str(prim.GetPath())
    if not prim.HasAPI(UsdPhysics.CollisionAPI):
        continue
    if "_foot" not in path and "base_link" not in path:
        continue
    mat, _ = UsdShade.MaterialBindingAPI(prim).ComputeBoundMaterial("physics")
    key = str(mat.GetPath()) if mat else None
    if key in seen:
        continue
    attrs = {}
    if mat:
        for a in mat.GetPrim().GetAttributes():
            if "friction" in a.GetName().lower() or "restitution" in a.GetName().lower():
                attrs[a.GetName()] = a.Get()
    seen[key] = attrs
    print(path.split("/env_0/")[-1], "->", key, attrs)
