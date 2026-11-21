"""Check sensor -> obs -> ONNX -> final target and temporal action state."""
import argparse
import json
from pathlib import Path
import platform

import numpy as np
import onnxruntime as ort

from runtime import DEFAULT, load_session, observation, wrap

def verify(model,tape):
    with np.load(tape) as d:
        obs=observation(d["q_rad"],d["dq_rad_s"],d["gyro_rad_s"],d["quat_wxyz"],
                        d["command_si"],d["prior_target"])
        raw=load_session(model).run(["leg_action"],{"observation":obs.reshape(-1,58)})[0].reshape(obs.shape[:-1]+(12,))
        targets=wrap(d["prior_target"],raw,d["command_si"])
        valid=~d["reset"]
        result={"frames":obs.shape[0],"trials":obs.shape[1],"checked_targets":int(valid.sum()),
                "device":platform.machine(),"onnxruntime":ort.__version__}
        assert valid.any()
        nz=np.flatnonzero(valid.flatten())
        pairs=[
            ("observation",obs,d["observation"],3e-6,None),
            ("raw_action",raw,d["raw_action"],5e-5,None),
            ("next_target",targets[valid],d["next_target"][valid],1.5e-5,nz),
            ("leg_q_target",DEFAULT[:12]+targets[valid][:,:12],d["leg_q_target"][valid],1.5e-5,nz),
            ("wheel_dq_target",targets[valid][:,12:],d["wheel_dq_target"][valid],4e-6,nz),
            ("next_frame_prior",targets[:-1][valid[:-1]],d["prior_target"][1:][valid[:-1]],1.5e-5,np.flatnonzero(valid[:-1].flatten())),
        ]
        ok=True
        for name,actual,expected,tol,indices in pairs:
            delta=np.abs(actual-expected)
            idx=np.unravel_index(np.argmax(delta),delta.shape)
            if indices is not None:
                idx=(*divmod(int(indices[idx[0]]),obs.shape[1]),idx[-1])
            error=float(delta.max())
            ok&=bool(np.isfinite(error) and error<=tol)
            result[name]={"max_abs_error":error,"limit":tol,"worst_frame_trial_feature":tuple(map(int,idx))}
        result["pass"]=ok
        return result

if __name__=="__main__":
    p=argparse.ArgumentParser()
    p.add_argument("onnx",type=Path)
    p.add_argument("tape",type=Path)
    p.add_argument("--out",type=Path)
    args=p.parse_args()
    r=verify(args.onnx,args.tape)
    output=json.dumps(r,indent=2)+"\n"
    if args.out:
        args.out.write_text(output)
    print(output,end="")
    raise SystemExit(0 if r["pass"] else 1)
