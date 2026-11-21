"""Gain-wiring regression for validate_session.py p55g/p55s/p85dg/p85ds.

Synthesizes a minimal stand-trial session (LOW_SAMPLE state stream +
DEBUG_LOWCMD FixedStand rows at one leg PD pair) and checks that the
validator's C2-gains assertion follows the MODE's contract:
  fixture 55/2  -> p55g PASS, p85dg FAIL (off-contract)
  fixture 85/3.2 -> p85dg PASS, p85ds PASS, p55g FAIL
Run:  python p85d_validator_regression.py   (scratch, 2026-11-19)
"""
import os
import subprocess
import sys
import tempfile

sys.stdout.reconfigure(encoding='utf-8', errors='replace')
V = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                 '..', '..', '..', '02_现场操作',
                 'AS2W悬空负载对照与Hip扫角_20261119',
                 'scripts', 'validate_session.py')
V = os.path.normpath(V)
D = os.path.join(tempfile.gettempdir(), 'p85d_regr')
os.makedirs(D, exist_ok=True)


def low_sample(i):
    # LOW_SAMPLE,elapsed,packet_count,age,tick,mode_pr,mode_machine,
    # quat x4, gyro x3, 35x(q,dq,ddq,tau,mode,temp)
    f = ['LOW_SAMPLE', str(i * 2), str(i + 1), '1000', str(1000 + i),
         '0', '2', '1.0', '0.0', '0.0', '0.0', '0.0', '0.0', '0.0']
    for m in range(35):
        f += ['0.0', '0.0', '0.0', '0.0', '1' if m < 16 else '0', '30']
    return ','.join(f)


def debug_row(i, kp, kd):
    # DEBUG_LOWCMD,mono_ns,tick,state,mode_machine,
    # 16x(slot,mode,q,dq,kp,kd,tau) -- all rows state==2 (FixedStand hold)
    f = ['DEBUG_LOWCMD', str(i * 20000000), str(1000 + i), '2', '2']
    for ax in range(16):
        legs = ax < 12
        f += [str(ax), '1', '0.1', '0.0',
              str(float(kp)) if legs else '0.0',
              str(float(kd)) if legs else '2.0', '0.0']
    return ','.join(f)


def build(kp, kd):
    n = 6000  # 12 s at 2 ms
    with open(os.path.join(D, 'low_capture_fast.log'), 'w') as f:
        for i in range(n):
            f.write(low_sample(i) + '\n')
    with open(os.path.join(D, 'debug_lowcmd.csv'), 'w') as f:
        for i in range(n):
            f.write(debug_row(i, kp, kd) + '\n')


def run(mode):
    r = subprocess.run([sys.executable, V, '--mode', mode, '--dir', D,
                        '--fixture'],
                       capture_output=True, text=True,
                       encoding='utf-8', errors='replace')
    lines = (r.stdout + r.stderr).splitlines()
    g = [l for l in lines if 'C2-gains-seen' in l]
    v = [l for l in lines if l.startswith('verdict')]
    return (g[0] if g else 'NO C2-GAINS LINE'), (v[0] if v else '?'), r.returncode


for name, kp, kd, modes in [
        ('P55 55/2', 55, 2, ('p55g', 'p85dg')),
        ('P85D 85/3.2', 85, 3.2, ('p85dg', 'p85ds', 'p55g'))]:
    build(kp, kd)
    for mode in modes:
        g, v, rc = run(mode)
        print(f'fixture={name:12s} mode={mode:5s} rc={rc} | {g} | {v}')
