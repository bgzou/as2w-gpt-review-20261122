"""Convert the bundled URDF and author collision exclusions before simulation."""
from pathlib import Path
from isaaclab.app import AppLauncher


def main():
    app = AppLauncher(headless=True).app
    try:
        from isaaclab.sim.converters import UrdfConverter, UrdfConverterCfg
        from pxr import Usd
        from v2_collision import apply_collision_exclusions

        root = Path(__file__).resolve().parent
        cfg = UrdfConverterCfg(
            asset_path=str(root / "assets/as2w-t2-v2.urdf"),
            usd_dir=str(root / "generated-v2"), usd_file_name="as2w-t2-v2.usd",
            fix_base=False, merge_fixed_joints=True, joint_drive=None,
            force_usd_conversion=True,
        )
        converter = UrdfConverter(cfg)
        stage = Usd.Stage.Open(converter.usd_path)
        if apply_collision_exclusions(stage) != 1:
            raise RuntimeError("Expected exactly one robot in converted asset")
        output = root / "generated-v2/as2w-t2-v2/as2w-t2-v2.usd"
        output.parent.mkdir(parents=True, exist_ok=True)
        temporary = output.with_suffix(".tmp.usd")
        if not stage.Export(str(temporary), args={"format": "usdc"}):
            raise RuntimeError(f"USD export failed: {temporary}")
        from pxr import UsdPhysics
        exported = Usd.Stage.Open(str(temporary))
        pairs = [target for prim in exported.Traverse()
                 for target in UsdPhysics.FilteredPairsAPI(prim).GetFilteredPairsRel().GetTargets()
                 if prim.GetName() == "base_link" and target.name in ("FL_thigh", "FR_thigh")
                 and exported.GetPrimAtPath(target).IsValid()]
        if sorted(target.name for target in pairs) != ["FL_thigh", "FR_thigh"]:
            raise RuntimeError(f"Missing v2 collision exclusions in exported USD: {pairs}")
        temporary.replace(output)
        print(f"V2_USD_READY {output}", flush=True)
    finally:
        app.close()


if __name__ == "__main__":
    main()
