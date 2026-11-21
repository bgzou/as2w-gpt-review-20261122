"""Export the exact unnormalized deterministic actor from the PPO checkpoint."""
from pathlib import Path
import argparse

import onnx
import torch
from torch import nn

def load_actor(path):
    actor=nn.Sequential(nn.Linear(58,512),nn.ELU(),nn.Linear(512,256),nn.ELU(),
                        nn.Linear(256,128),nn.ELU(),nn.Linear(128,12))
    state=torch.load(path,weights_only=True,map_location="cpu")["actor_state_dict"]
    if set(state)-{"distribution.std_param"}!={"mlp."+k for k in actor.state_dict()}:
        raise ValueError("Unexpected policy architecture or observation normalization")
    actor.load_state_dict({k.removeprefix("mlp."):v for k,v in state.items() if k.startswith("mlp.")})
    return actor.eval()

def export(path,out):
    actor=load_actor(path)
    torch.onnx.export(actor,torch.zeros((1,58),dtype=torch.float32),str(out),
                      input_names=["observation"],output_names=["leg_action"],opset_version=17,
                      dynamo=False,do_constant_folding=True,
                      dynamic_axes={"observation":{0:"batch"},"leg_action":{0:"batch"}})
    model=onnx.load(str(out),load_external_data=False)
    onnx.checker.check_model(model)
    if any(x.data_location==onnx.TensorProto.EXTERNAL for x in model.graph.initializer):
        raise RuntimeError("External ONNX data unexpected")

if __name__=="__main__":
    p=argparse.ArgumentParser()
    p.add_argument("checkpoint",type=Path)
    p.add_argument("out",type=Path)
    a=p.parse_args()
    export(a.checkpoint,a.out)
