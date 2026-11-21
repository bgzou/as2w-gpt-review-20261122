import sys
from pathlib import Path
import torch
from policy import LIMIT, compose, gate, load_policy, read_actor
base_path, composite_path=map(Path,sys.argv[1:3])
base=read_actor(torch.load(base_path,map_location="cpu",weights_only=False)["actor_state_dict"],"cpu")
policy=load_policy(composite_path,"cpu")
torch.manual_seed(42)
obs=torch.randn(6144,58)
cmd=torch.tensor([[0.,0.,0.],[.6,0.,0.],[-.6,0.,0.],[0.,.3,0.],[0.,0.,.6],[.4,-.3,-.6]])
obs[:,:3]=cmd.repeat_interleave(1024,dim=0)
frozen=base(obs).clamp(-1,1)
actual=policy({"policy":obs})
assert torch.equal(actual[:3072], frozen[:3072])
assert torch.equal(compose(frozen,torch.zeros_like(frozen),gate(obs[:,:3])),frozen)
diff=(actual-frozen).abs()
assert torch.isfinite(actual).all() and 0<diff.max()<=LIMIT+1e-6
stress=compose(frozen,torch.randn_like(frozen)*100,gate(obs[:,:3]))
assert torch.equal(stress[:3072],frozen[:3072]) and (stress-frozen).abs().max()<=LIMIT+1e-6
print("locked_frames",3072,"max_locked_error",float(diff[:3072].max()),"max_step_delta_rad",float(diff.max()*.25),"stress_bound_rad",float((stress-frozen).abs().max()*.25))
