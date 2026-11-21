import sys, pathlib
ROOT = pathlib.Path('/tmp/as2w-unitreestyle')
sys.path.insert(0, str(ROOT / 'common'))
from isaaclab.app import AppLauncher
launcher = AppLauncher(headless=True)
app = launcher.app
try:
    import torch
    from isaaclab.scene import InteractiveScene
    from environment import AS2WCfg, AS2WSceneCfg, LEGS
    from isaaclab.utils import configclass
    @configclass
    class Cfg(AS2WCfg):
        scene = AS2WSceneCfg(num_envs=1, env_spacing=3., replicate_physics=True)
    cfg = Cfg(rough=False, randomize=False, seed=42)
    import isaaclab.sim as sim_utils
    cfg.scene.robot.spawn.joint_drive = sim_utils.UrdfFileCfg.JointDriveCfg(
        target_type='position',
        gains=sim_utils.UrdfFileCfg.JointDriveCfg.PDGainsCfg(stiffness=55., damping=2.))
    import isaaclab.sim as sim_utils
    sim = sim_utils.SimulationContext(cfg.sim)
    g = sim_utils.CuboidCfg(size=(6.,6.,.1), collision_props=sim_utils.CollisionPropertiesCfg(contact_offset=.005, rest_offset=0.),
                            rigid_props=sim_utils.RigidBodyPropertiesCfg(kinematic_enabled=True, disable_gravity=True))
    g.func('/World/ground', g, translation=(0.,0.,-.05))
    cfg.scene.terrain = None
    scene = InteractiveScene(cfg.scene)
    sim.reset()
    robot = scene['robot']
    names = robot.joint_names
    ids = [names.index(n) for n in LEGS]
    tgt = torch.tensor([[0.,0.,0.,0., .8,.8,.8,.8, -1.6,-1.6,-1.6,-1.6]], device=robot.device)
    for i in range(300):
        robot.set_joint_position_target(tgt, joint_ids=ids)
        sim.step()
        robot.update(cfg.sim.dt)
        if i in (0, 10, 100, 299):
            print('P', i, [round(float(x),3) for x in robot.data.joint_pos[0, ids]],
                  'tau', [round(float(x),2) for x in robot.data.applied_torque[0, ids]], flush=True)
except BaseException:
    import traceback; traceback.print_exc(); raise
finally:
    try: app.close()
    except SystemExit: pass
