#!/usr/bin/env python3
"""G3: MuJoCo wheel-collision-primitive x scenario x initial-condition matrix.

Question: does the wheel collision primitive (CYLINDER vs MESH convex hull)
change the static-equilibrium branch, and do initial conditions select among
branches?

Matrix (18 runs): scenario {S suspended, G ground-P55, GO ground-official}
  x primitive {cyl, mesh} x init {real, nearflat, crouch}.

Primitive isolation note: the delivered A_C1_base MJCF already carries a
*visual* foot mesh geom that is also collidable (contype=1) IN ADDITION to the
cylinder, so the "cylinder" baseline is really cylinder+mesh.  For a clean 2x2
we build:
  M_cylwheel  = A_C1_base with the visual foot mesh made non-colliding
                (contype=0 conaffinity=0) -> ONLY the cylinder collides.
  M_meshwheel = A_C1_base with the cylinder geom removed -> ONLY the foot STL
                mesh collides.  MuJoCo 3.12 mesh collision = CONVEX HULL.
Suspended variants additionally drop the floor geom and the freejoint (base
welded) so only gravity-hold torques are measured.

Reuses f2_ablation_mujoco.py settle contract: explicit PD tau=kp*(tgt-q)-kd*dq,
kp 55 kd 2 (G) / kp 200 kd 4 (GO), effort clips hip60/thigh60/calf90, wheel
velocity servo tau=clip(2*(-dq),-15,15), dt 0.002, 20000 steps, final 20% window.

Run: /home/robot/z00945176/isaac_env/venv312/bin/python g3_mujoco_matrix.py
"""
import csv
import json
import pathlib
import re

import mujoco
import numpy as np

REPO = pathlib.Path('/home/robot/z00945176/Rob2Nav')
RUNS = REPO / 'runs/as2w_v2_simreal_rootcause_20261119'
ABL = RUNS / 'mujoco_abl'
BASE_XML = ABL / 'A_C1_base' / 'as2w_scene.xml'
NPZ = RUNS / 'qc' / 'a1_timelines' / 'P55G2.npz'

LEGS = ['FL_hip', 'FR_hip', 'RL_hip', 'RR_hip',
        'FL_thigh', 'FR_thigh', 'RL_thigh', 'RR_thigh',
        'FL_calf', 'FR_calf', 'RL_calf', 'RR_calf']
WHE = ['FL_foot', 'FR_foot', 'RL_foot', 'RR_foot']
R_WHEEL = 0.086


def real_init_q():
    z = np.load(NPZ)
    t, q = z['t'], z['q']
    m = (t >= 8.0) & (t <= 19.0)
    return [round(float(x), 6) for x in q[m].mean(0)]


INITS = {
    'real': real_init_q(),
    'nearflat': [0.03] * 4 + [0.5] * 4 + [-1.2] * 4,
    'crouch': [0.21] * 4 + [1.2] * 4 + [-2.4] * 4,
}

SCEN = {
    'G': dict(tgt=[0, 0, 0, 0, 0.8, 0.8, 0.8, 0.8, -1.6, -1.6, -1.6, -1.6],
              kp=55.0, kd=2.0, ground=True),
    'GO': dict(tgt=[-0.15, 0.15, -0.15, 0.15, 0.6, 0.6, 0.6, 0.6,
                    -1.2, -1.2, -1.2, -1.2],
               kp=200.0, kd=4.0, ground=True),
    'S': dict(tgt=[0, 0, 0, 0, 0.8, 0.8, 0.8, 0.8, -1.6, -1.6, -1.6, -1.6],
              kp=55.0, kd=2.0, ground=False),
}


def build_xml(primitive, suspended):
    txt = BASE_XML.read_text()
    if primitive == 'cyl':
        # neutralise the collidable visual foot mesh: only the cylinder collides
        for leg in ('FL', 'FR', 'RL', 'RR'):
            old = f'<geom mesh="{leg}_foot" rgba="0.2 0.8 0.2 1"/>'
            new = (f'<geom mesh="{leg}_foot" rgba="0.2 0.8 0.2 1" '
                   f'contype="0" conaffinity="0"/>')
            assert old in txt, old
            txt = txt.replace(old, new)
    elif primitive == 'mesh':
        # remove the cylinder collision geom: only the foot mesh collides
        for leg in ('FL', 'FR', 'RL', 'RR'):
            pat = re.compile(r'[ \t]*<geom name="' + leg +
                             r'_wheel_coll"[^>]*/>\n')
            txt, n = pat.subn('', txt)
            assert n == 1, (leg, n)
    else:
        raise ValueError(primitive)
    if suspended:
        txt, n = re.subn(r'[ \t]*<geom name="floor"[^>]*/>\n', '', txt)
        assert n == 1
        txt, n = re.subn(r'[ \t]*<freejoint name="root"/>\n', '', txt)
        assert n == 1
    name = f'M_{primitive}wheel' + ('_susp' if suspended else '')
    dd = ABL / name
    dd.mkdir(parents=True, exist_ok=True)
    lnk = dd / 'meshes'
    if not lnk.exists():
        lnk.symlink_to(BASE_XML.parent / 'meshes')
    xp = dd / 'as2w_scene.xml'
    xp.write_text(txt)
    return xp


def lowest_wheel_z(m, d):
    """Lowest world z over all wheel collision geoms (mesh vertices / cylinder)."""
    lo = 1e9
    for w in WHE:
        bid = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, w)
        for g in range(m.ngeom):
            if m.geom_bodyid[g] != bid:
                continue
            if m.geom_contype[g] == 0 and m.geom_conaffinity[g] == 0:
                continue
            gt = m.geom_type[g]
            if gt == mujoco.mjtGeom.mjGEOM_MESH:
                mid = m.geom_dataid[g]
                va = m.mesh_vertadr[mid]
                vn = m.mesh_vertnum[mid]
                verts = m.mesh_vert[va:va + vn]
                R = d.geom_xmat[g].reshape(3, 3)
                world = verts @ R.T + d.geom_xpos[g]
                lo = min(lo, float(world[:, 2].min()))
            elif gt == mujoco.mjtGeom.mjGEOM_CYLINDER:
                # axis horizontal (euler x=90deg): lowest = centre - radius
                lo = min(lo, float(d.geom_xpos[g][2] - m.geom_size[g][0]))
            else:
                lo = min(lo, float(d.geom_xpos[g][2] - m.geom_rbound[g]))
    return lo


def run(xml_path, scen, init_name):
    cfg = SCEN[scen]
    tgt = np.array(cfg['tgt'], dtype=float)
    kp, kd = cfg['kp'], cfg['kd']
    m = mujoco.MjModel.from_xml_path(str(xml_path))
    m.actuator_gainprm[:, :] = 0.0
    m.actuator_biasprm[:, :] = 0.0
    d = mujoco.MjData(m)
    jid = {n: mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, n) for n in LEGS + WHE}
    qadr = {n: m.jnt_qposadr[jid[n]] for n in LEGS + WHE}
    vadr = {n: m.jnt_dofadr[jid[n]] for n in LEGS + WHE}
    kp_w = {'hip': 60.0, 'thigh': 60.0, 'calf': 90.0}
    has_free = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, 'root') >= 0

    d.qpos[:] = 0
    if has_free:
        d.qpos[3] = 1.0  # identity quat, pitch 0
    q0 = INITS[init_name]
    for i, n in enumerate(LEGS):
        d.qpos[qadr[n]] = q0[i]
    for n in WHE:
        d.qpos[qadr[n]] = 0.0
    mujoco.mj_forward(m, d)
    if cfg['ground'] and has_free:
        lo = lowest_wheel_z(m, d)
        d.qpos[2] += (0.002 - lo)  # lowest wheel point ~2 mm above ground
        mujoco.mj_forward(m, d)

    nstep = 20000
    rec_from = int(nstep * 0.8)
    tau_h = [[] for _ in range(12)]
    q_h = [[] for _ in range(12)]
    fn_h = {w: [] for w in WHE}
    nc_h = {w: [] for w in WHE}
    nc_max = {w: 0 for w in WHE}
    cpos_h = {w: [] for w in WHE}
    pitch_h = []
    dqmax_h = []
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
        idx = [vadr[n] for n in LEGS + WHE]
        d.qfrc_applied[idx] = tau[idx]
        mujoco.mj_step(m, d)
        dqmax_h.append(max(abs(d.qvel[vadr[n]]) for n in LEGS))
        if step >= rec_from:
            mujoco.mj_forward(m, d)
            if has_free:
                qq = d.qpos[3:7]
                pitch_h.append(np.arcsin(np.clip(
                    2 * (qq[0] * qq[2] - qq[3] * qq[1]), -1, 1)))
            else:
                pitch_h.append(0.0)  # welded base
            for i, n in enumerate(LEGS):
                tau_h[i].append(tau[vadr[n]])
                q_h[i].append(d.qpos[qadr[n]])
            for w in WHE:
                fsum = 0.0
                npts = 0
                for ci in range(d.ncon):
                    g1, g2 = d.contact[ci].geom1, d.contact[ci].geom2
                    if m.geom_bodyid[g1] == wheel_bid[w] or m.geom_bodyid[g2] == wheel_bid[w]:
                        f6 = np.zeros(6)
                        mujoco.mj_contactForce(m, d, ci, f6)
                        fsum += f6[0]
                        npts += 1
                        cpos_h[w].append([float(x) for x in d.contact[ci].pos])
                fn_h[w].append(fsum)
                nc_h[w].append(npts)
                nc_max[w] = max(nc_max[w], npts)

    dqmax = np.array(dqmax_h)
    over = np.where(dqmax > 0.2)[0]
    settle_time = round(float(over[-1] * m.opt.timestep), 2) if over.size else 0.0
    pitch = round(float(np.degrees(np.mean(pitch_h))), 2) if pitch_h else float('nan')
    hip_tau = [round(float(np.mean(h)), 2) for h in tau_h[:4]]
    hip_q = [round(float(np.mean(h)), 3) for h in q_h[:4]]
    Fn = [round(float(np.mean(fn_h[w])), 1) for w in WHE]
    ncon = [round(float(np.mean(nc_h[w])), 2) for w in WHE]
    cpos = {w: (np.mean(np.array(cpos_h[w]), 0).round(4).tolist()
                if cpos_h[w] else None) for w in WHE}
    if cfg['ground']:
        settle_ok = bool(all(f > 5.0 for f in Fn) and dqmax[-1] < 0.2
                         and np.isfinite(pitch))
    else:
        settle_ok = bool(dqmax[-1] < 0.2 and np.isfinite(pitch))
    return dict(
        engine='mujoco', scenario=scen, init_condition=init_name,
        wheel_primitive='cyl' if 'cylwheel' in str(xml_path) else 'mesh',
        hip_tau_Nm=hip_tau, hip_q=hip_q,
        pitch_deg=pitch,
        Fn=Fn, n_contacts_per_wheel=ncon,
        n_contacts_final=[int(nc_h[w][-1]) if nc_h[w] else 0 for w in WHE],
        n_contacts_max=[int(nc_max[w]) for w in WHE],
        settle_ok=settle_ok, settle_time_s=settle_time,
        quat_wxyz=[round(float(x), 5) for x in d.qpos[3:7]] if has_free else None,
        contact_pos_world=cpos,
        thigh_q=[round(float(np.mean(h)), 3) for h in q_h[4:8]],
        calf_q=[round(float(np.mean(h)), 3) for h in q_h[8:12]],
        thigh_tau=[round(float(np.mean(h)), 2) for h in tau_h[4:8]],
        calf_tau=[round(float(np.mean(h)), 2) for h in tau_h[8:12]],
    )


def main():
    rows = []
    for scen in ('S', 'G', 'GO'):
        for prim in ('cyl', 'mesh'):
            xp = build_xml(prim, suspended=(scen == 'S'))
            for init_name in ('real', 'nearflat', 'crouch'):
                r = run(xp, scen, init_name)
                rows.append(r)
                print(f"{scen:2s} {prim:4s} {init_name:8s} pitch={r['pitch_deg']:7.2f} "
                      f"hip_q={r['hip_q']} Fn={r['Fn']} ncon={r['n_contacts_per_wheel']} "
                      f"settle_ok={r['settle_ok']}", flush=True)
    # CSV
    fields = ['engine', 'scenario', 'init_condition', 'wheel_primitive',
              'hip_tau_Nm', 'hip_q', 'pitch_deg', 'Fn',
              'n_contacts_per_wheel', 'settle_ok', 'notes']
    with open(RUNS / 'tables' / 'G3_ENGINE_PRIMITIVE_MATRIX.csv', 'w',
              newline='') as f:
        w = csv.DictWriter(f, fieldnames=fields, extrasaction='ignore')
        w.writeheader()
        for r in rows:
            w.writerow({**{k: r[k] for k in fields if k != 'notes'},
                        'hip_tau_Nm': '/'.join(f'{x:.2f}' for x in r['hip_tau_Nm']),
                        'hip_q': '/'.join(f'{x:.3f}' for x in r['hip_q']),
                        'Fn': '/'.join(f'{x:.1f}' for x in r['Fn']),
                        'n_contacts_per_wheel': '/'.join(f'{x:.2f}' for x in r['n_contacts_per_wheel']),
                        'notes': ('mesh=convex hull' if r['wheel_primitive'] == 'mesh'
                                  else 'pure cylinder; visual mesh non-colliding')})
    # JSON
    with open(RUNS / 'tables' / 'G3_ENGINE_PRIMITIVE_MATRIX.json', 'w') as f:
        json.dump(rows, f, indent=1)
    print('wrote G3_ENGINE_PRIMITIVE_MATRIX.csv/.json', len(rows), 'rows')


if __name__ == '__main__':
    main()
