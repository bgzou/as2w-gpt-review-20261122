#!/usr/bin/env python3
"""Phase I5 + I1(MuJoCo side): Go2-W contact-configuration variants on MuJoCo.

I5 variants (all G scenario, real init, explicit PD kp55/kd2 per G3 f2 settle
contract, M_cylwheel derived XMLs -- cylinder is the ONLY wheel collider):
  W_A  baseline: condim=4, pyramidal cone, impratio=1 (default),
        friction "1.0 0.05 0.001"  (== G3 M_cylwheel G/real; harness replay)
  W_B  only cone="elliptic"
  W_C  only impratio="100"   (pyramidal cone: expected no effect -- recorded)
  W_D  condim=6 + elliptic + impratio=100  (Go2-W solver combo, AS2-W friction)
  W_E  W_D + Go2-W official wheel friction "0.8 0.02 0.01"

I1 MuJoCo control (fair-comparison gap: G3 PhysX used ImplicitActuatorCfg):
  W_I  implicit position/velocity actuators (kp=55/kv=2 legs, kv=2 wheels,
        force ranges 60/60/90 & 15) driven via ctrl targets, implicitfast
        integrator -- vs W_A explicit qfrc_applied PD @500 Hz.

Metrics (I6): hip_q, hip_tau, 12-joint q, pitch, per-wheel Fn/ncon,
settle_ok/settle_time.  Compared against P55G2 real window (8,19) s anchor.

Run: /home/robot/z00945176/isaac_env/venv312/bin/python i5_gow_contact_variants.py
"""
import csv
import json
import pathlib

import mujoco
import numpy as np

REPO = pathlib.Path('/home/robot/z00945176/Rob2Nav')
RUNS = REPO / 'runs/as2w_v2_simreal_rootcause_20261119'
ABL = RUNS / 'mujoco_abl'
BASE_XML = ABL / 'M_cylwheel' / 'as2w_scene.xml'
NPZ = RUNS / 'qc' / 'a1_timelines' / 'P55G2.npz'

LEGS = ['FL_hip', 'FR_hip', 'RL_hip', 'RR_hip',
        'FL_thigh', 'FR_thigh', 'RL_thigh', 'RR_thigh',
        'FL_calf', 'FR_calf', 'RL_calf', 'RR_calf']
WHE = ['FL_foot', 'FR_foot', 'RL_foot', 'RR_foot']
R_WHEEL = 0.086
TGT = np.array([0, 0, 0, 0, 0.8, 0.8, 0.8, 0.8, -1.6, -1.6, -1.6, -1.6])
KP, KD = 55.0, 2.0


def real_anchor():
    z = np.load(NPZ)
    t, q, tau, pitch = z['t'], z['q'], z['tau'], z['pitch']
    m = (t >= 8.0) & (t <= 19.0)
    return dict(q0=[round(float(x), 6) for x in q[m][0]],
                q_mean=q[m].mean(0), tau_mean=tau[m].mean(0),
                pitch_deg=float(np.degrees(pitch[m].mean())))


ANC = real_anchor()


def build_xml(name, cone=None, impratio=None, condim=None, wheel_fric=None,
              implicit_act=False):
    """Derive a variant XML from M_cylwheel. Returns path."""
    txt = BASE_XML.read_text()
    opt = f'<option timestep="0.002" integrator="implicitfast" gravity="0 0 -9.81"/>'
    attrs = ''
    if cone:
        attrs += f' cone="{cone}"'
    if impratio:
        attrs += f' impratio="{impratio}"'
    if attrs:
        txt = txt.replace(opt, f'<option timestep="0.002" integrator="implicitfast"'
                               f' gravity="0 0 -9.81"{attrs}/>')
    if condim:
        txt = txt.replace('<geom condim="4" friction="1.0 0.05 0.001"/>',
                          f'<geom condim="{condim}" friction="1.0 0.05 0.001"/>')
    if wheel_fric:
        for leg in ('FL', 'FR', 'RL', 'RR'):
            old = (f'<geom name="{leg}_wheel_coll" pos="')
            assert old in txt
            # add friction attr before the trailing slash of this geom line
            import re
            pat = re.compile(r'(<geom name="' + leg +
                             r'_wheel_coll"[^>]*?)(/>)')
            txt, n = pat.subn(r'\1' + f' friction="{wheel_fric}"' + r'\2', txt)
            assert n == 1, (leg, n)
    if implicit_act:
        # rewrite the actuator block to the explicit-baseline contract:
        # legs position kp=55 kv=2 (forcerange 60/60/90), wheels velocity kv=2
        import re
        txt = re.sub(r'(<position name="[A-Z]{2}_hip_act"[^>]*?)kp="60" kv="2"',
                     r'\1kp="55" kv="2"', txt)
        txt = re.sub(r'(<position name="[A-Z]{2}_thigh_act"[^>]*?)kp="60" kv="2"',
                     r'\1kp="55" kv="2"', txt)
        txt = re.sub(r'(<position name="[A-Z]{2}_calf_act"[^>]*?)kp="90" kv="3"',
                     r'\1kp="55" kv="2"', txt)
        txt = re.sub(r'(<velocity name="[A-Z]{2}_foot_act"[^>]*?)kv="8"',
                     r'\1kv="2"', txt)
        assert 'kp="55" kv="2"' in txt and 'kv="2" forcerange="-15 15"' in txt
    dd = ABL / name
    dd.mkdir(parents=True, exist_ok=True)
    lnk = dd / 'meshes'
    if not lnk.exists():
        lnk.symlink_to(BASE_XML.parent / 'meshes')
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
            if m.geom_type[g] == mujoco.mjtGeom.mjGEOM_CYLINDER:
                lo = min(lo, float(d.geom_xpos[g][2] - m.geom_size[g][0]))
            else:
                lo = min(lo, float(d.geom_xpos[g][2] - m.geom_rbound[g]))
    return lo


def run(xml_path, variant, implicit_act):
    m = mujoco.MjModel.from_xml_path(str(xml_path))
    if not implicit_act:
        m.actuator_gainprm[:, :] = 0.0   # neutralize factory actuators
        m.actuator_biasprm[:, :] = 0.0
    d = mujoco.MjData(m)
    jid = {n: mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, n) for n in LEGS + WHE}
    qadr = {n: m.jnt_qposadr[jid[n]] for n in LEGS + WHE}
    vadr = {n: m.jnt_dofadr[jid[n]] for n in LEGS + WHE}
    kp_w = {'hip': 60.0, 'thigh': 60.0, 'calf': 90.0}
    # actuator ctrl addresses for implicit variant
    act_qadr = {}
    if implicit_act:
        for i, n in enumerate(LEGS):
            a = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_ACTUATOR, f'{n}_act')
            act_qadr[i] = a
        whe_acts = [mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_ACTUATOR,
                                      f'{w}_act') for w in WHE]

    d.qpos[:] = 0
    d.qpos[3] = 1.0  # identity quat
    for i, n in enumerate(LEGS):
        d.qpos[qadr[n]] = ANC['q0'][i]
    mujoco.mj_forward(m, d)
    d.qpos[2] += (0.002 - lowest_wheel_z(m, d))
    mujoco.mj_forward(m, d)

    nstep = 20000
    rec_from = int(nstep * 0.8)
    tau_h = [[] for _ in range(12)]
    q_h = [[] for _ in range(12)]
    fn_h = {w: [] for w in WHE}
    nc_h = {w: [] for w in WHE}
    pitch_h = []
    dqmax_h = []
    wheel_bid = {w: mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, w) for w in WHE}
    for step in range(nstep):
        if implicit_act:
            for i, n in enumerate(LEGS):
                d.ctrl[act_qadr[i]] = TGT[i]
            for a in whe_acts:
                d.ctrl[a] = 0.0
            mujoco.mj_step(m, d)
            if step >= rec_from:
                for i, n in enumerate(LEGS):
                    tau_h[i].append(d.actuator_force[act_qadr[i]])
        else:
            tau = np.zeros(m.nv)
            for i, n in enumerate(LEGS):
                kind = n.split('_')[1]
                e = TGT[i] - d.qpos[qadr[n]]
                ed = -d.qvel[vadr[n]]
                tau[vadr[n]] = np.clip(KP * e + KD * ed, -kp_w[kind], kp_w[kind])
            for n in WHE:
                tau[vadr[n]] = np.clip(2.0 * (-d.qvel[vadr[n]]), -15, 15)
            d.qfrc_applied[:] = 0
            idx = [vadr[n] for n in LEGS + WHE]
            d.qfrc_applied[idx] = tau[idx]
            mujoco.mj_step(m, d)
            if step >= rec_from:
                for i, n in enumerate(LEGS):
                    tau_h[i].append(tau[vadr[n]])
        dqmax_h.append(max(abs(d.qvel[vadr[n]]) for n in LEGS))
        if step >= rec_from:
            mujoco.mj_forward(m, d)
            qq = d.qpos[3:7]
            pitch_h.append(np.arcsin(np.clip(
                2 * (qq[0] * qq[2] - qq[3] * qq[1]), -1, 1)))
            for i, n in enumerate(LEGS):
                q_h[i].append(d.qpos[qadr[n]])
            for w in WHE:
                fsum, npts = 0.0, 0
                for ci in range(d.ncon):
                    g1, g2 = d.contact[ci].geom1, d.contact[ci].geom2
                    if m.geom_bodyid[g1] == wheel_bid[w] or \
                       m.geom_bodyid[g2] == wheel_bid[w]:
                        f6 = np.zeros(6)
                        mujoco.mj_contactForce(m, d, ci, f6)
                        fsum += f6[0]
                        npts += 1
                fn_h[w].append(fsum)
                nc_h[w].append(npts)

    dqmax = np.array(dqmax_h)
    over = np.where(dqmax > 0.2)[0]
    settle_time = round(float(over[-1] * m.opt.timestep), 2) if over.size else 0.0
    pitch = round(float(np.degrees(np.mean(pitch_h))), 2)
    hip_q = [round(float(np.mean(h)), 3) for h in q_h[:4]]
    hip_tau = [round(float(np.mean(h)), 2) for h in tau_h[:4]]
    Fn = [round(float(np.mean(fn_h[w])), 1) for w in WHE]
    ncon = [round(float(np.mean(nc_h[w])), 2) for w in WHE]
    settle_ok = bool(all(f > 5.0 for f in Fn) and dqmax[-1] < 0.2)
    q_err_hip = [round(float(np.mean(q_h[i]) - ANC['q_mean'][i]), 3)
                 for i in range(4)]
    tau_err_hip = [round(float(np.mean(tau_h[i]) - ANC['tau_mean'][i]), 2)
                   for i in range(4)]
    return dict(
        variant=variant, control='implicit_act' if implicit_act else 'explicit_pd',
        hip_q=hip_q, hip_tau_Nm=hip_tau, pitch_deg=pitch, Fn=Fn,
        n_contacts_per_wheel=ncon, settle_ok=settle_ok,
        settle_time_s=settle_time,
        hip_q_err_vs_real=q_err_hip, hip_tau_err_vs_real=tau_err_hip,
        pitch_err_deg=round(pitch - ANC['pitch_deg'], 2),
        q12=[round(float(np.mean(h)), 3) for h in q_h],
        tau12=[round(float(np.mean(h)), 2) for h in tau_h],
        q12_err_rms=round(float(np.sqrt(np.mean(
            [(np.mean(q_h[i]) - ANC['q_mean'][i]) ** 2 for i in range(12)]))), 4),
    )


VARIANTS = [
    # name, xml kwargs, implicit_act
    ('W_A', dict(), False),
    ('W_B', dict(cone='elliptic'), False),
    ('W_C', dict(impratio='100'), False),
    ('W_D', dict(cone='elliptic', impratio='100', condim='6'), False),
    ('W_E', dict(cone='elliptic', impratio='100', condim='6',
                 wheel_fric='0.8 0.02 0.01'), False),
    ('W_I', dict(implicit_act=True), True),
]


def main():
    rows = []
    for name, kw, impl in VARIANTS:
        xp = build_xml(name, **kw)
        r = run(xp, name, impl)
        rows.append(r)
        print(f"{name} [{r['control']}] pitch={r['pitch_deg']:7.2f} "
              f"hip_q={r['hip_q']} hip_tau={r['hip_tau_Nm']} Fn={r['Fn']} "
              f"ncon={r['n_contacts_per_wheel']} qErrRMS={r['q12_err_rms']} "
              f"settle_ok={r['settle_ok']}", flush=True)
    with open(RUNS / 'tables' / 'I5_GOW_CONTACT_VARIANTS.csv', 'w',
              newline='') as f:
        keys = ['variant', 'control', 'hip_q', 'hip_tau_Nm', 'pitch_deg', 'Fn',
                'n_contacts_per_wheel', 'settle_ok', 'settle_time_s',
                'hip_q_err_vs_real', 'hip_tau_err_vs_real', 'pitch_err_deg',
                'q12_err_rms', 'q12', 'tau12']
        w = csv.DictWriter(f, fieldnames=keys, extrasaction='ignore')
        w.writeheader()
        for r in rows:
            w.writerow(r)
    with open(RUNS / 'tables' / 'I5_GOW_CONTACT_VARIANTS.json', 'w') as f:
        json.dump(dict(rows=rows,
                       real_anchor=dict(
                           hip_q=[round(float(x), 4) for x in ANC['q_mean'][:4]],
                           hip_tau=[round(float(x), 2) for x in ANC['tau_mean'][:4]],
                           pitch_deg=round(ANC['pitch_deg'], 2))), f, indent=1)
    print('wrote I5_GOW_CONTACT_VARIANTS.{csv,json}')


if __name__ == '__main__':
    main()
