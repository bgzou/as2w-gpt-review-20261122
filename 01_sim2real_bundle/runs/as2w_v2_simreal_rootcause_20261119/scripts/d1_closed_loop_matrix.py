#!/usr/bin/env python3
"""Phase D: MuJoCo closed-loop matrix, stock (C0) vs wheel-center-corrected (C1).

C1 correction (H5 asset fix): add pos="0 +/-0.0546 0" to each *_wheel_coll
cylinder geom (FL/RL +0.0546, FR/RR -0.0546), moving the collision wheel to
the real wheel center (URDF/STL; track width 0.4400 vs stock 0.3308 m).

Scenarios (closed-loop PD at real contract gains, qfrc_applied path):
  S  suspended: base kinematically held (strap), no contact, P55 target T2
  G  ground: free base, wheels on ground, P55 target T2
  GO ground official: official target @ kp200/kd4
Each run: settle, mean over last 20% window. Compare with measured means.

Outputs:
  runs/.../mujoco_c1/as2w_scene.xml  (corrected copy, meshes symlinked)
  runs/.../tables/MUJOCO_REAL_SIM_MATRIX.csv
Run: /home/robot/z00945176/isaac_env/venv312/bin/python
"""
import csv
import pathlib

import mujoco
import numpy as np

REPO = pathlib.Path('/home/robot/z00945176/Rob2Nav')
C0_XML = REPO / 'deliveries/as2w-t15-layered-delivery-20261117/simulation/mujoco/as2w_scene.xml'
RUNS = REPO / 'runs/as2w_v2_simreal_rootcause_20261119'
C1_DIR = RUNS / 'mujoco_c1'
C1_XML = C1_DIR / 'as2w_scene.xml'

LEGS = ['FL_hip', 'FR_hip', 'RL_hip', 'RR_hip',
        'FL_thigh', 'FR_thigh', 'RL_thigh', 'RR_thigh',
        'FL_calf', 'FR_calf', 'RL_calf', 'RR_calf']
WHE = ['FL_foot', 'FR_foot', 'RL_foot', 'RR_foot']
OFF_Y = {'FL': 0.0546, 'RL': 0.0546, 'FR': -0.0546, 'RR': -0.0546}
R_WHEEL = 0.086
REAL = {  # measured means (corrected V2 hip sign +1; see 01/04 docs)
    'S': dict(tau_hip=[2.771, -3.162, 2.960, -3.057], tag='P55S 25-90'),
    'G': dict(tau_hip=[-11.805, 11.695, -15.153, 15.188], tag='P55G2 8-19'),
    'GO': dict(tau_hip=[-4.350, 5.610, -5.786, 4.463], tag='OG 17-120'),
}


def build_c1():
    C1_DIR.mkdir(parents=True, exist_ok=True)
    lnk = C1_DIR / 'meshes'
    if not lnk.exists():
        lnk.symlink_to(C0_XML.parent / 'meshes')
    txt = C0_XML.read_text()
    for pre, oy in OFF_Y.items():
        old = f'name="{pre}_wheel_coll" type="cylinder"'
        new = f'name="{pre}_wheel_coll" pos="0 {oy} 0" type="cylinder"'
        assert old in txt, old
        txt = txt.replace(old, new)
    C1_XML.write_text(txt)
    return C1_XML


def run_case(xml_path, scen):
    m = mujoco.MjModel.from_xml_path(str(xml_path))
    # CRITICAL FIX: the shipped MJCF has 16 built-in position/velocity actuators
    # with ctrl=0 (leg kp 60/90 toward q=0). Left active they silently cancel any
    # externally applied PD torque (discovered: thigh settled at the balance of
    # two competing PDs, 44/115=0.3826). Neutralize them; drive via qfrc_applied.
    m.actuator_gainprm[:, :] = 0.0
    m.actuator_biasprm[:, :] = 0.0
    d = mujoco.MjData(m)
    jid = {n: mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, n) for n in LEGS + WHE}
    qadr = {n: m.jnt_qposadr[jid[n]] for n in LEGS + WHE}
    vadr = {n: m.jnt_dofadr[jid[n]] for n in LEGS + WHE}
    if scen == 'GO':
        # corrected V2 official model-coord target (hip sign +1): symmetric inward tuck
        tgt = np.array([-0.15, 0.15, -0.15, 0.15, 0.6, 0.6, 0.6, 0.6,
                        -1.2, -1.2, -1.2, -1.2])
        kp, kd = 200.0, 4.0
    else:
        tgt = np.array([0, 0, 0, 0, 0.8, 0.8, 0.8, 0.8, -1.6, -1.6, -1.6, -1.6])
        kp, kd = 55.0, 2.0
    kp_w = {'hip': 60.0, 'thigh': 60.0, 'calf': 90.0}
    nstep = 25000  # 50 s
    # init pose
    d.qpos[:] = 0
    d.qpos[3] = 1.0
    for i, n in enumerate(LEGS):
        d.qpos[qadr[n]] = tgt[i]
    if scen == 'S':
        # fixed-base XML (freejoint removed) emulates the strap properly.
        # Earlier version pinned qpos[:7] after each mj_step -- invalid: the base
        # is free DURING the step, giving incoherent dynamics. Superseded.
        z = np.load(RUNS / 'qc' / 'a1_timelines' / 'P55S.npz')
        msk = (z['t'] >= 25) & (z['t'] <= 90)
        for i, n in enumerate(LEGS):
            d.qpos[qadr[n]] = float(z['q'][msk][:, i].mean())
    else:
        mujoco.mj_forward(m, d)
        pts = [d.xpos[mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, w)]
               - np.array([0, 0, R_WHEEL]) for w in WHE]
        d.qpos[2] -= min(p[2] for p in pts)
    base0 = d.qpos[:7].copy()
    mujoco.mj_forward(m, d)
    tau_leg_hist = [[] for _ in range(12)]
    q_hist = [[] for _ in range(12)]
    fn_hist = {w: [] for w in WHE}
    pitch_hist, roll_hist, ncon_hist = [], [], []
    rec_from = int(nstep * 0.8)
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
            roll_hist.append(np.arctan2(2 * (qq[0] * qq[1] + qq[2] * qq[3]),
                                        1 - 2 * (qq[1] ** 2 + qq[2] ** 2)))
            pitch_hist.append(np.arcsin(np.clip(2 * (qq[0] * qq[2] - qq[3] * qq[1]), -1, 1)))
            ncon_hist.append(d.ncon)
            for i, n in enumerate(LEGS):
                tau_leg_hist[i].append(tau[vadr[n]])
                q_hist[i].append(d.qpos[qadr[n]])
            for w in WHE:
                fsum = 0.0
                for ci in range(d.ncon):
                    g1, g2 = d.contact[ci].geom1, d.contact[ci].geom2
                    if m.geom_bodyid[g1] == wheel_bid[w] or m.geom_bodyid[g2] == wheel_bid[w]:
                        f6 = np.zeros(6)
                        mujoco.mj_contactForce(m, d, ci, f6)
                        fsum += f6[0]
                fn_hist[w].append(fsum)
    res = dict(
        scen=scen, xml=xml_path.name,
        hip_tau=[round(float(np.mean(h)), 3) for h in tau_leg_hist[:4]],
        thigh_tau=[round(float(np.mean(h)), 3) for h in tau_leg_hist[4:8]],
        calf_tau=[round(float(np.mean(h)), 3) for h in tau_leg_hist[8:12]],
        hip_q=[round(float(np.mean(h)), 4) for h in q_hist[:4]],
        pitch_deg=round(float(np.degrees(np.mean(pitch_hist))), 2),
        roll_deg=round(float(np.degrees(np.mean(roll_hist))), 2),
        Fn=[round(float(np.mean(fn_hist[w])), 2) for w in WHE],
        ncon_mean=round(float(np.mean(ncon_hist)), 1))
    return res


def build_fixbase():
    """Fixed-base variants (freejoint removed) for the suspended S scenario."""
    outs = {}
    for tag, src in (('C0', C0_XML), ('C1', RUNS / 'mujoco_c1' / 'as2w_scene.xml')):
        dd = RUNS / f'mujoco_s{tag[1].lower()}'
        dd.mkdir(parents=True, exist_ok=True)
        xp = dd / 'as2w_scene.xml'
        xp.write_text(src.read_text().replace(
            '<freejoint name="root"/>',
            '<!-- root fixed for suspended strap scenario -->'))
        lnk = dd / 'meshes'
        if not lnk.exists():
            lnk.symlink_to(C0_XML.parent / 'meshes')
        outs[tag] = xp
    return outs


def main():
    c1 = build_c1()
    fb = build_fixbase()
    rows = []
    for scen in ('S', 'G', 'GO'):
        for label, xp in (('C0', C0_XML if scen != 'S' else fb['C0']),
                          ('C1', c1 if scen != 'S' else fb['C1'])):
            r = run_case(xp, scen)
            r['variant'] = label
            rows.append(r)
            print(label, scen, r)
    # CSV
    out = RUNS / 'tables' / 'MUJOCO_REAL_SIM_MATRIX.csv'
    fields = ['variant', 'scen', 'hip_tau_FL', 'hip_tau_FR', 'hip_tau_RL', 'hip_tau_RR',
              'thigh_tau', 'calf_tau', 'hip_q', 'pitch_deg', 'roll_deg', 'Fn', 'ncon_mean']
    with open(out, 'w', newline='') as f:
        w = csv.writer(f)
        w.writerow(fields)
        for r in rows:
            w.writerow([r['variant'], r['scen'], *r['hip_tau'],
                        str(r['thigh_tau']), str(r['calf_tau']), str(r['hip_q']),
                        r['pitch_deg'], r['roll_deg'], str(r['Fn']), r['ncon_mean']])
    print('wrote', out)


if __name__ == '__main__':
    main()
