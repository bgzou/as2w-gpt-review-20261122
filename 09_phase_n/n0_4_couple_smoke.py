#!/usr/bin/env python3
"""N0-4: MuJoCo contact-couple interface smoke test (v2).

The first version applied an external torque and read one contact's force; with a
box on a plane that mixes rigid-body tipping with the friction constraint.  This
version instead *prescribes* a small relative spin/roll rate each step (qvel
override) and reads the full contact wrench, which isolates the friction
constraint's moment channel.

Checks:
  * compiled geom condim / dim / the three friction numbers
  * condim=3/4/6 under a prescribed yaw rate (moment about the contact normal)
  * condim=3/4/6 under a prescribed roll rate (moment in the contact tangent plane)
  * per-contact dim and the 6 components from mj_contactForce
"""
import json
import pathlib

import mujoco
import numpy as np

OUT = pathlib.Path(__file__).resolve().parent.parent / 'results'
OUT.mkdir(parents=True, exist_ok=True)

TMPL = """<mujoco model="couple_smoke">
  <option timestep="0.001" integrator="implicitfast"/>
  <worldbody>
    <geom name="floor" type="plane" size="2 2 0.1" friction="1.0 0.010 0.005"/>
    <body name="blk" pos="0 0 0.52">
      <freejoint name="root"/>
      <geom name="blk" type="box" size="0.05 0.05 0.005" mass="1.0"
            condim="{condim}" friction="1.0 0.010 0.005"/>
    </body>
  </worldbody>
</mujoco>
"""


def contact_wrench(m, d):
    """Sum the 6-component contact wrench over all contacts, and list dims."""
    tot = np.zeros(6)
    dims = []
    for ci in range(d.ncon):
        c = d.contact[ci]
        f6 = np.zeros(6)
        mujoco.mj_contactForce(m, d, ci, f6)
        tot += f6
        dims.append(int(c.dim))
    return tot, dims


def run_case(condim, axis, rate):
    m = mujoco.MjModel.from_xml_string(TMPL.format(condim=condim))
    d = mujoco.MjData(m)
    sid = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_GEOM, 'blk')
    mujoco.mj_forward(m, d)
    for _ in range(3000):
        mujoco.mj_step(m, d)
    if d.ncon == 0:
        return dict(condim=condim, error='no contact at rest')
    rest, dims0 = contact_wrench(m, d)
    # prescribe the relative rate: axis 5 = yaw about z, 3 = roll about x
    for _ in range(3000):
        d.qvel[axis] = rate
        d.qvel[0:3] = 0.0          # no translation
        mujoco.mj_step(m, d)
    if d.ncon == 0:
        return dict(condim=condim, error='contact lost')
    w, dims = contact_wrench(m, d)
    return dict(
        condim=condim, axis=('yaw' if axis == 5 else 'roll'), rate=rate,
        ncon=int(d.ncon), dims=dims, dim_rest=dims0,
        normal_force=round(float(w[0]), 4),
        tangential=[round(float(w[1]), 5), round(float(w[2]), 5)],
        torsional_moment=round(float(w[3]), 6),
        rolling_moment=[round(float(w[4]), 6), round(float(w[5]), 6)],
        rest_normal=round(float(rest[0]), 4),
        friction_slide_torsion_roll=[float(x) for x in m.geom_friction[sid]],
        expected_torsion_limit=round(float(m.geom_friction[sid][1] * w[0]), 5),
        expected_roll_limit=round(float(m.geom_friction[sid][2] * w[0]), 5),
    )


def static_twist(condim, axis, delta):
    """Hold a prescribed yaw/roll offset and read the moment the contact exerts."""
    m = mujoco.MjModel.from_xml_string(TMPL.format(condim=condim))
    d = mujoco.MjData(m)
    mujoco.mj_forward(m, d)
    for _ in range(2000):
        mujoco.mj_step(m, d)
    if d.ncon == 0:
        return dict(condim=condim, error='no contact')
    q0 = d.qpos[3:7].copy()
    half = delta / 2.0
    s, c = np.sin(half), np.cos(half)
    if axis == 5:      # yaw about z
        q = np.array([c * q0[0] - s * q0[3], c * q0[1] - s * q0[2],
                      c * q0[2] + s * q0[1], c * q0[3] + s * q0[0]])
    else:              # roll about x
        q = np.array([c * q0[0] + s * q0[3], c * q0[1] - s * q0[2],
                      c * q0[2] + s * q0[1], c * q0[3] - s * q0[0]])
    q = q / np.linalg.norm(q)
    held = d.qpos.copy()
    held[3:7] = q
    for _ in range(2000):
        d.qpos[:] = held
        d.qvel[:] = 0.0
        mujoco.mj_step(m, d)
    w, dims = contact_wrench(m, d)
    return dict(condim=condim, axis=('yaw' if axis == 5 else 'roll'),
                delta_rad=delta, ncon=int(d.ncon), dims=dims,
                normal_force=round(float(w[0]), 4),
                tangential=[round(float(w[1]), 5), round(float(w[2]), 5)],
                torsional_moment=round(float(w[3]), 6),
                rolling_moment=[round(float(w[4]), 6), round(float(w[5]), 6)])


def main():
    info = {}
    for condim in (3, 4, 6):
        m = mujoco.MjModel.from_xml_string(TMPL.format(condim=condim))
        sid = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_GEOM, 'blk')
        info[condim] = dict(geom_condim=int(m.geom_condim[sid]),
                            friction=[float(x) for x in m.geom_friction[sid]],
                            friction_shape=list(m.geom_friction.shape),
                            condim_shape=list(m.geom_condim.shape))
    rows = []
    for condim in (3, 4, 6):
        for axis, nm in ((5, 'yaw'), (3, 'roll')):
            for rate in (0.0, 0.01, 0.1, 1.0):
                r = run_case(condim, axis, rate)
                r['case'] = nm
                rows.append(r)
                print(json.dumps(r, ensure_ascii=False), flush=True)
    static_rows = []
    for condim in (3, 4, 6):
        for axis, nm in ((5, 'yaw'), (3, 'roll')):
            for delta in (0.002, 0.01, 0.05, 0.2, 1.0):
                r = static_twist(condim, axis, delta)
                r['case'] = nm
                static_rows.append(r)
                print('STATIC ' + json.dumps(r, ensure_ascii=False), flush=True)
    out = dict(schema='n0_4_contact_couple_smoke_v2',
               mujoco_version=mujoco.__version__,
               geom_info=info, rows=rows, static_rows=static_rows,
               note=('geom/friction has three numbers: sliding coefficient, '
                     'torsional friction length (m), rolling friction length (m). '
                     'condim=4 adds the torsional (about-normal) moment; condim=6 '
                     'adds the rolling (tangent-plane) moment.  The prescribed-rate '
                     'rows show a rate-dependent (impedance) moment; the static rows '
                     'hold a fixed offset and show the quasi-static moment.'))
    (OUT / 'N0_4_contact_couple_smoke.json').write_text(
        json.dumps(out, ensure_ascii=False, indent=1))
    print('wrote', OUT / 'N0_4_contact_couple_smoke.json')


if __name__ == '__main__':
    main()
