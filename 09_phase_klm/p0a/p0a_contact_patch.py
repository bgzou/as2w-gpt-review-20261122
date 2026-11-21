#!/usr/bin/env python3
"""P0-A: finite contact-patch candidate study on MuJoCo (Phase K).

Baseline: archived M_cylwheel scene (C1 wheel-centre geometry, single cylinder
collider per wheel, condim=4, friction "1.0 0.05 0.001", dt 0.002, implicitfast).
Driver: explicit PD kp=55 kd=2 (tau limits hip/thigh 60, calf 90), wheel -2*dq
clipped +-15, 20000 steps, metrics over the last 20 % -- identical to
i5_gow_contact_variants.py W_A.  C0 reproduces the archived W_A numbers
bit-for-bit on MuJoCo 4.0.1, so all candidates are compared against the same
in-engine C0 rather than against a cross-version reference.

Candidates
  C0    single cylinder (one contact point per wheel)
  C1    3 coaxial cylinders across the tyre width (half-width 8.667 mm each,
        y = -13 / 0 / +13 mm around the wheel centre): a 3-point lateral
        discretisation of the same outer profile -- union == original cylinder.
  C1f   5 coaxial cylinders (half-width 5.2 mm): discretisation convergence.
  C3    C1 + soft contact solref ("0.01 1") on the wheel geoms only.
  C0M+  C0 + external pure moment about x at every foot, +Fn*b/2 (the largest
        couple a normal-pressure distribution over the 52 mm tyre width can
        generate).  This is an *external wrench counterfactual*, not a contact
        model; it bounds what any patch-width normal redistribution could do.
  C0M-  same with the opposite sign.

A 2-row (2x3) patch is NOT built: rows along the rolling direction require
either eccentric colliders or a flattened outer profile, both of which change
the wheel geometry and would confound "contact count" with "profile".  The
limitation is recorded instead of faking the geometry.
"""
import hashlib
import json
import pathlib

import mujoco
import numpy as np

HERE = pathlib.Path(__file__).resolve().parent
ROOT = HERE.parent
BASE = ROOT / 'input' / 'as2w_scene_base.xml'
NPZ = ROOT / 'input' / 'P55G2.npz'
SCENES = ROOT / 'scenes'
RESULTS = ROOT / 'results'

LEGS = ['FL_hip', 'FR_hip', 'RL_hip', 'RR_hip',
        'FL_thigh', 'FR_thigh', 'RL_thigh', 'RR_thigh',
        'FL_calf', 'FR_calf', 'RL_calf', 'RR_calf']
WHE = ['FL_foot', 'FR_foot', 'RL_foot', 'RR_foot']
OFF_Y = {'FL': 0.0546, 'RL': 0.0546, 'FR': -0.0546, 'RR': -0.0546}
R_WHEEL = 0.086
HALF_W = 0.026
TGT_G = np.array([0, 0, 0, 0, 0.8, 0.8, 0.8, 0.8, -1.6, -1.6, -1.6, -1.6])
TGT_GO = np.array([-0.15, 0.15, -0.15, 0.15, 0.6, 0.6, 0.6, 0.6,
                   -1.2, -1.2, -1.2, -1.2])


def anchor():
    z = np.load(NPZ)
    t, q, tau, pitch = z['t'], z['q'], z['tau'], z['pitch']
    m = (t >= 8.0) & (t <= 19.0)
    return dict(q0=[float(x) for x in q[m][0]],
                q_mean=q[m].mean(0), tau_mean=tau[m].mean(0),
                pitch_deg=float(np.degrees(pitch[m].mean())))


ANC = anchor()


def cylinder_geoms(prefix, oy, n, half_w, solref=None):
    ys = [(-HALF_W + half_w) + i * (2 * HALF_W - 2 * half_w) / (n - 1) for i in range(n)] if n > 1 else [0.0]
    out = []
    for i, dy in enumerate(ys):
        sr = f' solref="{solref}"' if solref else ''
        out.append(
            f'<geom name="{prefix}_wheel_p{i}" pos="0 {oy + dy:.6f} 0" type="cylinder" '
            f'size="0.086 {half_w:.6f}" euler="1.5708 0 0" rgba="0 1 0 0.3"{sr}/>')
    return out


def build(name, nseg=1, solref=None):
    txt = BASE.read_text()
    half_w = HALF_W if nseg == 1 else HALF_W / nseg
    for leg in OFF_Y:
        pat = (f'<geom name="{leg}_wheel_coll" pos="0 {OFF_Y[leg]:.4f} 0" type="cylinder" '
               f'size="0.086 0.026" euler="1.5708 0 0" rgba="0 1 0 0.3"/>')
        assert pat in txt, pat
        txt = txt.replace(pat, '\n      '.join(cylinder_geoms(leg, OFF_Y[leg], nseg, half_w, solref)))
    dd = SCENES / name
    dd.mkdir(parents=True, exist_ok=True)
    lnk = dd / 'meshes'
    if not lnk.exists():
        lnk.symlink_to(ROOT / 'input' / 'meshes')
    xp = dd / 'as2w_scene.xml'
    xp.write_text(txt)
    return xp


def lowest_wheel_z(m, d):
    lo = 1e9
    for w in WHE:
        bid = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, w)
        for g in range(m.ngeom):
            if m.geom_bodyid[g] != bid:
                continue
            if m.geom_contype[g] == 0 and m.geom_conaffinity[g] == 0:
                continue
            if m.geom_type[g] in (mujoco.mjtGeom.mjGEOM_CYLINDER,
                                  mujoco.mjtGeom.mjGEOM_SPHERE):
                lo = min(lo, float(d.geom_xpos[g][2] - m.geom_size[g][0]))
            else:
                lo = min(lo, float(d.geom_xpos[g][2] - m.geom_rbound[g]))
    return lo


def run(xml_path, scen, kp, couple=None):
    m = mujoco.MjModel.from_xml_path(str(xml_path))
    m.actuator_gainprm[:, :] = 0.0
    m.actuator_biasprm[:, :] = 0.0
    d = mujoco.MjData(m)
    jid = {n: mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, n) for n in LEGS + WHE}
    qadr = {n: m.jnt_qposadr[jid[n]] for n in LEGS + WHE}
    vadr = {n: m.jnt_dofadr[jid[n]] for n in LEGS + WHE}
    kp_w = {'hip': 60.0, 'thigh': 60.0, 'calf': 90.0}
    tgt = TGT_GO if scen == 'GO' else TGT_G

    d.qpos[:] = 0
    d.qpos[3] = 1.0
    for i, n in enumerate(LEGS):
        d.qpos[qadr[n]] = ANC['q0'][i]
    mujoco.mj_forward(m, d)
    d.qpos[2] += (0.002 - lowest_wheel_z(m, d))
    mujoco.mj_forward(m, d)
    if couple is not None:
        for i, w in enumerate(WHE):
            bid = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, w)
            d.xfrc_applied[bid, 3] = float(couple[i])

    nstep = 20000
    rec_from = int(nstep * 0.8)
    tau_h = [[] for _ in range(12)]
    con_h = [[] for _ in range(12)]
    q_h = [[] for _ in range(12)]
    fn_h = {w: [] for w in WHE}
    nc_h = {w: [] for w in WHE}
    pen_h = {w: [] for w in WHE}
    pitch_h, roll_h = [], []
    dqmax_h = []
    wheel_bid = {w: mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, w) for w in WHE}
    for step in range(nstep):
        tau = np.zeros(m.nv)
        for i, n in enumerate(LEGS):
            kind = n.split('_')[1]
            e = tgt[i] - d.qpos[qadr[n]]
            ed = -d.qvel[vadr[n]]
            tau[vadr[n]] = np.clip(kp * e + 2.0 * ed, -kp_w[kind], kp_w[kind])
        for n in WHE:
            tau[vadr[n]] = np.clip(2.0 * (-d.qvel[vadr[n]]), -15, 15)
        d.qfrc_applied[:] = 0
        idx = [vadr[n] for n in LEGS + WHE]
        d.qfrc_applied[idx] = tau[idx]
        mujoco.mj_step(m, d)
        dqmax_h.append(max(abs(d.qvel[vadr[n]]) for n in LEGS))
        if step >= rec_from:
            mujoco.mj_forward(m, d)
            qq = d.qpos[3:7]
            pitch_h.append(np.arcsin(np.clip(2 * (qq[0] * qq[2] - qq[3] * qq[1]), -1, 1)))
            roll_h.append(np.arctan2(2 * (qq[0] * qq[1] + qq[2] * qq[3]),
                                     1 - 2 * (qq[1] ** 2 + qq[2] ** 2)))
            for i, n in enumerate(LEGS):
                tau_h[i].append(tau[vadr[n]])
                q_h[i].append(d.qpos[qadr[n]])
                con_h[i].append(d.qfrc_constraint[vadr[n]])
            for w in WHE:
                fsum, npts = 0.0, 0
                pen = 0.0
                for ci in range(d.ncon):
                    c = d.contact[ci]
                    if m.geom_bodyid[c.geom1] != wheel_bid[w] and \
                       m.geom_bodyid[c.geom2] != wheel_bid[w]:
                        continue
                    f6 = np.zeros(6)
                    mujoco.mj_contactForce(m, d, ci, f6)
                    fsum += f6[0]
                    npts += 1
                    pen = min(pen, float(c.dist))
                fn_h[w].append(fsum)
                nc_h[w].append(npts)
                pen_h[w].append(pen)
    dqmax = np.array(dqmax_h)
    over = np.where(dqmax > 0.2)[0]
    settle_time = round(float(over[-1] * m.opt.timestep), 2) if over.size else 0.0

    def mean(x):
        return float(np.mean(x)) if len(x) else float('nan')

    res = dict(
        scene=scen, kp=kp,
        mujoco_version=mujoco.__version__,
        timestep=float(m.opt.timestep),
        hip_q=[round(mean(h), 4) for h in q_h[:4]],
        hip_tau_Nm=[round(mean(h), 2) for h in tau_h[:4]],
        hip_constraint_tau_Nm=[round(mean(h), 3) for h in con_h[:4]],
        thigh_q=[round(mean(h), 4) for h in q_h[4:8]],
        calf_q=[round(mean(h), 4) for h in q_h[8:12]],
        thigh_tau_Nm=[round(mean(h), 2) for h in tau_h[4:8]],
        calf_tau_Nm=[round(mean(h), 2) for h in tau_h[8:12]],
        pitch_deg=round(float(np.degrees(mean(pitch_h))), 2),
        roll_deg=round(float(np.degrees(mean(roll_h))), 2),
        Fn=[round(mean(fn_h[w]), 2) for w in WHE],
        Fn_sum=round(float(sum(mean(fn_h[w]) for w in WHE)), 2),
        ncon_per_wheel=[round(mean(nc_h[w]), 2) for w in WHE],
        penetration_mm=[round(mean(pen_h[w]) * 1000.0, 3) for w in WHE],
        settle_time_s=settle_time,
        dqmax_last=round(float(dqmax[-1]), 4),
        dqmax_p99=round(float(np.percentile(dqmax, 99)), 4),
        hip_q_err_vs_real=[round(mean(q_h[i]) - ANC['q_mean'][i], 4) for i in range(4)],
        hip_tau_err_vs_real=[round(mean(tau_h[i]) - ANC['tau_mean'][i], 2) for i in range(4)],
        pitch_err_deg=round(float(np.degrees(mean(pitch_h))) - ANC['pitch_deg'], 2),
    )
    return res


CANDIDATES = {
    'C0': dict(nseg=1),
    'C0Mp': dict(nseg=1),
    'C0Mn': dict(nseg=1),
    'C1': dict(nseg=3),
    'C1f': dict(nseg=5),
    'C3a': dict(nseg=3, solref='0.02 1'),
    'C3b': dict(nseg=3, solref='0.05 1'),
}
SCEN = {'C0': ['G', 'GO'], 'C0Mp': ['G'], 'C0Mn': ['G'],
        'C1': ['G', 'GO'], 'C1f': ['G'], 'C3a': ['G'], 'C3b': ['G']}
COUPLE_SIGN = {'C0Mp': +1.0, 'C0Mn': -1.0}


def main():
    RESULTS.mkdir(parents=True, exist_ok=True)
    rows = []
    base_fn = None
    for name, kw in CANDIDATES.items():
        xp = build(name, **kw)
        for scen in SCEN[name]:
            kp = 200.0 if scen == 'GO' else 55.0
            moment = None
            if name in COUPLE_SIGN and base_fn is not None:
                moment = [COUPLE_SIGN[name] * f * HALF_W for f in base_fn]
            r = run(xp, scen, kp, couple=moment)
            if name == 'C0' and scen == 'G':
                base_fn = r['Fn']
            r['candidate'] = name
            r['xml_sha'] = hashlib.sha256(xp.read_bytes()).hexdigest()
            rows.append(r)
            print(json.dumps(r, ensure_ascii=False), flush=True)
    (RESULTS / 'p0a_contact_patch.json').write_text(
        json.dumps(dict(
            schema='p0a_contact_patch_v1',
            mujoco_version=mujoco.__version__,
            real_anchor=dict(hip_q=[round(float(x), 4) for x in ANC['q_mean'][:4]],
                             hip_tau=[round(float(x), 2) for x in ANC['tau_mean'][:4]],
                             pitch_deg=round(ANC['pitch_deg'], 3)),
            pairs=[r for r in rows]), ensure_ascii=False, indent=1))
    print('wrote', RESULTS / 'p0a_contact_patch.json')


if __name__ == '__main__':
    main()
