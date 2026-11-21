"""Emit rows consumed by the independent C++ controller math replay."""
import sys
import numpy as np

with np.load(sys.argv[1]) as tape:
    keys=("q_rad","dq_rad_s","gyro_rad_s","quat_wxyz","command_si","prior_target","raw_action",
          "observation","next_target")
    arrays=np.concatenate([tape[k] for k in keys],axis=-1)
    np.savetxt(sys.stdout,arrays[~tape["reset"]],fmt="%.9g")
