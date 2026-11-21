import torch
from torch import nn

LIMIT = 0.24  # normalized action, before the 0.25 rad scale

def read_actor(state, device):
    layers = []
    for i in (0, 2, 4, 6):
        w = state[f"mlp.{i}.weight"]
        layer = nn.Linear(w.shape[1], w.shape[0])
        layer.load_state_dict({"weight": w, "bias": state[f"mlp.{i}.bias"]})
        if layers:
            layers.append(nn.ELU())
        layers.append(layer)
    return nn.Sequential(*layers).to(device).requires_grad_(False).eval()

def gate(command):
    return torch.maximum(command[:, 1:2].abs() / .15, command[:, 2:3].abs() / .35).clamp(max=1.)

def compose(base, latent, strength):
    return (base.clamp(-1., 1.) + strength * LIMIT * latent.tanh()).clamp(-1., 1.)

def load_policy(path, device, profile=None):
    data = torch.load(path, map_location="cpu", weights_only=False)
    meta = data["infos"]["as2w_residual_v1"]
    assert meta["limit"] == LIMIT
    if profile is not None and profile != meta.get("profile","original"):
        raise ValueError("Checkpoint and environment posture differ")
    base = read_actor(meta["base_actor"], device)
    residual = read_actor(data["actor_state_dict"], device)
    def policy(obs):
        frame = obs["policy"]
        return compose(base(frame), residual(frame), gate(frame[:, :3]))
    return policy
