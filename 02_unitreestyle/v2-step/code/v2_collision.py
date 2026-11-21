"""Author the two v2 body-pair exclusions on spawned Isaac robot links."""
def apply_collision_exclusions(stage):
    from pxr import UsdPhysics
    count = 0
    for prim in stage.Traverse():
        if prim.GetName() != "base_link" or not prim.HasAPI(UsdPhysics.RigidBodyAPI):
            continue
        from pxr import Usd
        links = {link.GetName(): link for link in Usd.PrimRange(prim)
                 if link.HasAPI(UsdPhysics.RigidBodyAPI)}
        targets = [links[name] for name in ("FL_thigh", "FR_thigh")]
        relation = UsdPhysics.FilteredPairsAPI.Apply(prim).CreateFilteredPairsRel()
        for target in targets:
            relation.AddTarget(target.GetPath())
        count += 1
    if count == 0:
        raise RuntimeError("No base_link rigid body found for v2 collision filter")
    print(f"V2_COLLISION_FILTER authored_robot_count={count}", flush=True)
    return count
