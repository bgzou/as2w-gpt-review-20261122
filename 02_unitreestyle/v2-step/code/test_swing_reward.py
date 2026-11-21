import json,torch
from pathlib import Path
from swing_reward import SwingReward
n=5
s=SwingReward(n,'cpu',json.loads(Path(__file__).with_name('calibration.json').read_text()))
mode=torch.tensor([0,1,2,3,-1]);sw=torch.zeros(n,dtype=torch.bool)
pos=torch.zeros(n,4,3);q=torch.zeros(n,4,4);q[...,3]=1
zero=torch.zeros_like(pos);normal=zero.clone();normal[...,2]=1
force=torch.full((n,4),30.)
def step(load,height=0):
 force.fill_(load);pos[...,2]=height
 r=s(mode,sw,force,pos,q,zero,zero,zero,normal,.02)
 return r
step(30)
offset=-s.last_clearance[0,0].item()
for _ in range(4):step(30,offset)
events=torch.zeros(n)
for _ in range(6): events+=step(0,offset+.04)[0]
for _ in range(3): events+=step(30,offset)[0]
assert torch.allclose(events,torch.tensor([1.,1.,1.,1.,0.])),events
# Second then third swing: per-wheel cap prevents farming.
for cycle in range(2):
 total=torch.zeros(n)
 for _ in range(6):total+=step(0,offset+.04)[0]
 for _ in range(3):total+=step(30,offset)[0]
 assert torch.allclose(total,torch.tensor([1.,1.,1.,1.,0.]) if cycle==0 else torch.zeros(n)),total
s.reset(slice(None))
for _ in range(4):step(30,offset)
total=torch.zeros(n)
for _ in range(6):total+=step(0,offset+.005)[0]
for _ in range(3):total+=step(30,offset)[0]
assert not total.any(),'low clearance incorrectly rewarded'
for _ in range(150):r=step(30,offset)
assert torch.all(r[1][:4]>.99) and r[1][4]==0,r[1]
sw[:]=True
r=step(30,offset);assert not r[1].any(),'switch grace not reset'
print('PASS: all 4 modes/all 4 wheels, vx isolation, credit cap, low clearance rejected, missing coverage, switch reset')
