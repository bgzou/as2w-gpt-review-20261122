"""Replay T2 FixedStand in the training USD without policy inference."""
import argparse
import json
import traceback
from pathlib import Path
from isaaclab.app import AppLauncher


def report_materials():
    import omni.usd
    from pxr import Usd, UsdPhysics, UsdShade

    stage = omni.usd.get_context().get_stage()
    for prim in stage.Traverse(Usd.TraverseInstanceProxies()):
        path = str(prim.GetPath())
        if not prim.HasAPI(UsdPhysics.CollisionAPI):
            continue
        if not (path.startswith("/World/ground") or
                ("/env_0/" in path and "_foot" in path)):
            continue
        material, _ = UsdShade.MaterialBindingAPI(prim).ComputeBoundMaterial("physics")
        attributes = {}
        if material:
            for attr in material.GetPrim().GetAttributes():
                if "friction" in attr.GetName().lower():
                    attributes[attr.GetName()] = str(attr.Get())
        print("T2_MATERIAL", json.dumps({"collider": path,
              "material": str(material.GetPath()) if material else None,
              "attributes": attributes}), flush=True)


def make_tire_material_variant(source, output, friction):
    from pxr import Usd, UsdPhysics, UsdShade

    stage = Usd.Stage.Open(str(source))
    # Edit a diagnostic copy. USD forbids authoring on instance proxies.
    while instances := [prim for prim in stage.Traverse() if prim.IsInstance()]:
        for prim in instances:
            if not prim.SetInstanceable(False):
                raise RuntimeError(f"Could not expand instance {prim.GetPath()}")
    material = UsdShade.Material.Define(stage, stage.GetDefaultPrim().GetPath().AppendChild("ProbeTireMaterial"))
    physics = UsdPhysics.MaterialAPI.Apply(material.GetPrim())
    physics.CreateStaticFrictionAttr(friction)
    physics.CreateDynamicFrictionAttr(friction)
    bound = 0
    for prim in stage.Traverse():
        path = str(prim.GetPath())
        if not prim.HasAPI(UsdPhysics.CollisionAPI) or "_foot" not in path:
            continue
        UsdShade.MaterialBindingAPI.Apply(prim).Bind(material, materialPurpose="physics")
        bound += 1
    if bound != 4:
        raise RuntimeError(f"Expected four wheel colliders, found {bound}")
    output.parent.mkdir(parents=True, exist_ok=True)
    if not stage.Export(str(output)):
        raise RuntimeError(f"Could not export {output}")
    return output


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--asset", required=True, type=Path)
    parser.add_argument("--out", required=True, type=Path)
    parser.add_argument("--field-init", action="store_true")
    parser.add_argument("--ground-friction", type=float)
    parser.add_argument("--material-only", action="store_true")
    parser.add_argument("--tire-friction", type=float)
    args = parser.parse_args()
    args.num_envs, args.dr_level, args.seed, args.terrain = 4, 0, 319, "flat"
    app = AppLauncher(headless=True).app
    env = None
    try:
        import numpy as np
        import torch
        from environment import LEGS, WHEELS, tensor
        from task import build_task

        if args.tire_friction is not None:
            variant = args.out.with_suffix(".usd")
            if variant.resolve() == args.asset.resolve() or variant.exists():
                raise ValueError(f"Diagnostic asset must use a new path: {variant}")
            args.asset = make_tire_material_variant(args.asset, variant, args.tire_friction)
        env = build_task(args)
        if args.tire_friction is not None:
            report_materials()
        if args.material_only:
            report_materials()
            return
        env.exercise = torch.zeros((env.num_envs, 3), device=env.device)
        env.reset()
        data = env.robot.data
        reference = tensor(data.default_joint_pos)[:, env.leg_ids]
        action = torch.zeros_like(reference)
        labels = ["nominal"] * env.num_envs
        if args.field_init:
            table = json.loads(Path(__file__).with_name("t2_reset_poses.json").read_text())
            first = {row["session"]: row for row in reversed(table)}
            selected = [first[key] for key in ("182802", "185930", "190012")]
            ids = torch.arange(1, env.num_envs, device=env.device)
            pose = tensor(data.root_state_w)[ids, :7].clone()
            quat = reference.new_tensor([row["quat_wxyz"] for row in selected])
            pose[:, 3:7] = quat[:, [1, 2, 3, 0]]  # Isaac 3.0 native root pose is xyzw.
            joint = tensor(data.joint_pos)[ids].clone()
            joint[:, env.leg_ids] = reference.new_tensor([row["q"] for row in selected])
            env.robot.write_root_pose_to_sim(pose, env_ids=ids)
            env.robot.write_joint_state_to_sim(joint, torch.zeros_like(joint), env_ids=ids)
            labels[1:] = [row["session"] for row in selected]
        samples = []
        wheel_forces = []
        root_quaternions = []
        with torch.inference_mode():
            for _ in range(round(6. / env.step_dt)):
                _, _, terminated, timeout, _ = env.step(action)
                if (terminated | timeout).any():
                    raise RuntimeError("T2 failed to hold; episode reset during probe")
                q = tensor(data.joint_pos)[:, env.leg_ids]
                dq = tensor(data.joint_vel)[:, env.leg_ids]
                pd = 55. * (reference - q) - 2. * dq
                samples.append(torch.cat((q, dq, pd, env.last_normal_force,
                                          tensor(data.projected_gravity_b)), dim=1).cpu().numpy())
                force = tensor(env.contact.data.net_forces_w)[:, env.sensor_wheels]
                wheel_forces.append(force.cpu().numpy().copy())
                root_quaternions.append(tensor(data.root_quat_w).cpu().numpy().copy())
        args.out.parent.mkdir(parents=True, exist_ok=True)
        np.savez_compressed(args.out, samples=np.stack(samples), period_s=env.step_dt,
                            joints=LEGS, wheels=WHEELS, asset=str(args.asset), labels=labels,
                            wheel_forces_world_n=np.stack(wheel_forces),
                            root_quaternion_world_xyzw=np.stack(root_quaternions))
        print(f"T2_PROBE_OK {args.out}", flush=True)
    except Exception:
        traceback.print_exc()
        raise
    finally:
        if env is not None:
            env.close()
        app.close()


if __name__ == "__main__":
    main()
