#!/usr/bin/env python3
"""Phase E audit: controlled MuJoCo ablations (independent copies, formal
delivery assets untouched).

Axis isolated one at a time from the C1 baseline (official mass/CoM, C1 wheel
geometry, floor friction 1.0/0.05/0.001, condim 4, dt 0.002, kd 2):
  geom C0, friction 0.05/0.3/0.8, condim 3/6, solref hard/soft, dt 0.005,
  kd 0, load-v0 mass+CoM, wider tyre patch.

Outputs: runs/.../tables/F_E_MUJOCO_ABLATION.csv
Run: /home/robot/z00945176/isaac_env/venv312/bin/python
"""
import csv
import pathlib

import mujoco
import numpy as np

REPO = pathlib.Path('/home/robot/z00945176/Rob2Nav')
C0_XML = REPO / 'deliveries/as2w-t15-layered-delivery-20261117/simulation/mujoco/as2w_scene.xml'
RUNS = REPO / 'runs/as2w_v2_simreal_rootcause_20261119'
ABL = RUNS / 'mujoco_abl'
LEGS = ['FL_hip', 'FR_hip', 'RL_hip', 'RR_hip',
        'FL_thigh', 'FR_thigh', 'RL_thigh', 'RR_thigh',
        'FL_calf', 'FR_calf', 'RL_calf', 'RR_calf']
WHE = ['FL_foot', 'FR_foot', 'RL_foot', 'RR_foot']
OFF_Y = {'FL': 0.0546, 'RL': 0.0546, 'FR': -0.0546, 'RR': -0.0546}
R_WHEEL = 0.086
OLD_INERTIAL = ('<inertial pos="0.035002 -0.0007562 0.017521" mass="10.08"\n'
                '                fullinertia="0.05069483 0.13822065 0.1505824 -0.0002906 -0.02480435 0.00027717"/>')
NEW_INERTIAL = ('<inertial pos="0.020002 -0.0007562 0.017521" mass="11.592"\n'
                '                fullinertia="0.0582990545 0.1589537475 0.17316976 -0.00033419 -0.0285250025 0.0003187455"/>')


def make_xml(name, geom_c1=True, friction=None, condim=None, solref=None,
             dt=None, mass_loadv0=False, wheel_half_len=None, condim6=False):
    txt = C0_XML.read_text()
    if geom_c1:
        for pre, oy in OFF_Y.items():
            old = f'name="{pre}_wheel_coll" type="cylinder"'
            new = f'name="{pre}_wheel_coll" pos="0 {oy} 0" type="cylinder"'
            assert old in txt
            txt = txt.replace(old, new)
    if friction is not None:
        txt = txt.replace('friction="1.0 0.05 0.001"',
                          f'friction="{friction} 0.05 0.001"')
    if condim is not None:
        txt = txt.replace('condim="4"', f'condim="{condim}"')
    if condim6:
        txt = txt.replace('condim="4" friction="1.0 0.05 0.001"',
                          'condim="6" friction="1.0 0.05 0.001"')
    if solref is not None:
        txt = txt.replace('<geom condim="4" friction="1.0 0.05 0.001"/>',
                          f'<geom condim="4" friction="1.0 0.05 0.001" solref="{solref}"/>')
        txt = txt.replace('<geom name="floor" type="plane" size="5 5 0.1" friction="1.0 0.05 0.001"/>',
                          f'<geom name="floor" type="plane" size="5 5 0.1" friction="1.0 0.05 0.001" solref="{solref}"/>')
    if dt is not None:
        txt = txt.replace('timestep="0.002"', f'timestep="{dt}"')
    if mass_loadv0:
        assert OLD_INERTIAL in txt
        txt = txt.replace(OLD_INERTIAL, NEW_INERTIAL)
    if wheel_half_len is not None:
        txt = txt.replace('size="0.086 0.026"', f'size="0.086 {wheel_half_len}"')
    dd = ABL / name
    dd.mkdir(parents=True, exist_ok=True)
    lnk = dd / 'meshes'
    if not lnk.exists():
        lnk.symlink_to(C0_XML.parent / 'meshes')
    xp = dd / 'as2w_scene.xml'
    xp.write_text(txt)
    return xp


def run(xml_path, scen, kd=2.0, nstep=20000):
    m = mujoco.MjModel.from_xml_path(str(xml_path))
    m.actuator_gainprm[:, :] = 0.0
    m.actuator_biasprm[:, :] = 0.0
    d = mujoco.MjData(m)
    jid = {n: mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, n) for n in LEGS + WHE}
    qadr = {n: m.jnt_qposadr[jid[n]] for n in LEGS + WHE}
    vadr = {n: m.jnt_dofadr[jid[n]] for n in LEGS + WHE}
    if scen == 'GO':
        tgt = np.array([-0.15, 0.15, -0.15, 0.15, 0.6, 0.6, 0.6, 0.6,
                        -1.2, -1.2, -1.2, -1.2])
        kp = 200.0
    else:
        tgt = np.array([0, 0, 0, 0, 0.8, 0.8, 0.8, 0.8, -1.6, -1.6, -1.6, -1.6])
        kp = 55.0
    kp_w = {'hip': 60.0, 'thigh': 60.0, 'calf': 90.0}
    d.qpos[:] = 0
    d.qpos[3] = 1.0
    for i, n in enumerate(LEGS):
        d.qpos[qadr[n]] = tgt[i]
    mujoco.mj_forward(m, d)
    # settle height: lowest wheel bottom to z=0 (naive projection, as d1)
    pts = [d.xpos[mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, w)] - np.array([0, 0, R_WHEEL])
           for w in WHE]
    d.qpos[2] -= min(p[2] for p in pts)
    mujoco.mj_forward(m, d)
    rec_from = int(nstep * 0.8)
    tau_h = [[] for _ in range(12)]
    q_h = [[] for _ in range(12)]
    fn_h = {w: [] for w in WHE}
    pitch_h, roll_h = [], []
    wheel_bid = {w: mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, w) for w in WHE}
    for step in range(nstep):
        tau = np.zeros(m.nv)
        for i, n in enumerate(LEGS):
            kind = n.split('_')[1]
            e = tgt[i] - d.qpos[qadr[n]]
            ed = -d.qvel[vadr[n]]
            tau[vadr[n]] = np.clip(kp * e + kd * ed, -kp_w[kind], kp_w[kind])
        for n in WHE:
            tau[vadr[n]] = np.clip(2.0 * (-d.qvel[vadr[n]]), -15, 15)
        d.qfrc_applied[:] = 0
        d.qfrc_applied[[vadr[n] for n in LEGS + WHE]] = tau[[vadr[n] for n in LEGS + WHE]]
        mujoco.mj_step(m, d)
        if step >= rec_from:
            mujoco.mj_forward(m, d)
            qq = d.qpos[3:7]
            roll_h.append(np.arctan2(2 * (qq[0] * qq[1] + qq[2] * qq[3]),
                                     1 - 2 * (qq[1] ** 2 + qq[2] ** 2)))
            pitch_h.append(np.arcsin(np.clip(2 * (qq[0] * qq[2] - qq[3] * qq[1]), -1, 1)))
            for i, n in enumerate(LEGS):
                tau_h[i].append(tau[vadr[n]])
                q_h[i].append(d.qpos[qadr[n]])
            for w in WHE:
                fsum = 0.0
                for ci in range(d.ncon):
                    g1, g2 = d.contact[ci].geom1, d.contact[ci].geom2
                    if m.geom_bodyid[g1] == wheel_bid[w] or m.geom_bodyid[g2] == wheel_bid[w]:
                        f6 = np.zeros(6)
                        mujoco.mj_contactForce(m, d, ci, f6)
                        fsum += f6[0]
                fn_h[w].append(fsum)
    return dict(
        hip_tau=[round(float(np.mean(h)), 2) for h in tau_h[:4]],
        hip_q=[round(float(np.mean(h)), 3) for h in q_h[:4]],
        thigh_tau=[round(float(np.mean(h)), 2) for h in tau_h[4:8]],
        calf_tau=[round(float(np.mean(h)), 2) for h in tau_h[8:12]],
        pitch=round(float(np.degrees(np.mean(pitch_h))), 2),
        roll=round(float(np.degrees(np.mean(roll_h))), 2),
        Fn=[round(float(np.mean(fn_h[w])), 1) for w in WHE],
        ncon=round(float(np.mean([1])), 1))


VARIANTS = [
    ('A_C1_base', dict()),
    ('B_C0_geom', dict(geom_c1=False)),
    ('C_fric0.05', dict(friction=0.05)),
    ('C_fric0.3', dict(friction=0.3)),
    ('C_fric0.8', dict(friction=0.8)),
    ('C_condim3', dict(condim=3)),
    ('C_condim6_torsion', dict(condim6=True)),
    ('D_solref_hard', dict(solref='0.002 1')),
    ('D_solref_soft', dict(solref='0.02 1')),
    ('D_dt0.005', dict(dt=0.005)),
    ('PD_kd0', dict()),
    ('E_loadv0_mass_com', dict(mass_loadv0=True)),
    ('F_wide_patch', dict(wheel_half_len=0.05)),
]


def main():
    rows = []
    for name, kw in VARIANTS:
        xp = make_xml(name, **kw)
        kd = 0.0 if name == 'PD_kd0' else 2.0
        r = run(xp, 'G', kd=kd)
        r.update(variant=name, scen='G')
        rows.append(r)
        print(name, r)
    # GO for the key axes only
    for name in ('A_C1_base', 'B_C0_geom', 'C_fric0.05', 'C_fric0.8', 'E_loadv0_mass_com'):
        xp = ABL / name / 'as2w_scene.xml'
        r = run(xp, 'GO')
        r.update(variant=name, scen='GO')
        rows.append(r)
        print(name, 'GO', r)
    fields = ['variant', 'scen', 'hip_tau', 'hip_q', 'thigh_tau', 'calf_tau',
              'pitch', 'roll', 'Fn']
    with open(RUNS / 'tables' / 'F_E_MUJOCO_ABLATION.csv', 'w', newline='') as f:
        w = csv.DictWriter(f, fieldnames=fields, extrasaction='ignore')
        w.writeheader()
        for r in rows:
            w.writerow({k: (str(v) if isinstance(v, list) else v) for k, v in r.items()})
    print('wrote F_E_MUJOCO_ABLATION.csv')


if __name__ == '__main__':
    main()
