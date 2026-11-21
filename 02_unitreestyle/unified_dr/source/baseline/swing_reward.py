"""Per-wheel contact events and loaded lateral slip on the flat training tiles."""
import torch
from measure import (MODES, WHEELS, PROFILE, CONTACT_OFF,
                     CONTACT_ON, OFF_FRAMES, ON_FRAMES)

class SwingReward:
    def __init__(self, n, device, calibration):
        self.params={}
        for field in ("eligible","min_s","max_s","min_m","max_m","min_interval_s","gap_budget_s"):
            self.params[field]=torch.as_tensor([[calibration[m][w].get(field,0.)
                                                for w in WHEELS] for m in MODES],device=device)
        self.mode=torch.full((n,),-1,device=device,dtype=torch.long)
        size=(n,4)
        self.off=torch.zeros(size,device=device,dtype=torch.int32)
        self.on=torch.zeros_like(self.off)
        self.air=torch.zeros_like(self.off)
        self.peak=torch.zeros(size,device=device)
        self.gap=torch.zeros_like(self.peak)
        self.armed=torch.zeros(size,device=device,dtype=torch.bool)
        self.seen=torch.zeros_like(self.armed)
        self.profile=torch.as_tensor(PROFILE,device=device)
        self.wheels=torch.arange(4,device=device)
        self.up=torch.tensor([0.,0.,1.],device=device)[None,None,:]

    def reset(self, ids):
        for t in (self.off,self.on,self.air,self.peak,self.gap,self.armed,self.seen):
            t[ids]=0
        self.mode[ids]=-1

    def __call__(self, mode, switched, force, position, quat, lin, ang, ground, normal, dt):
        active=(mode>=0)[:,None]
        changed=(switched | (mode!=self.mode))[:,None]
        self.mode.copy_(mode)
        params={k:v[mode.clamp_min(0)] for k,v in self.params.items()}
        eligible=(params["eligible"]>0)&active
        x,y,z,w=quat.unbind(-1)
        axis=torch.stack((2*(x*y-z*w),1-2*(x*x+z*z),2*(y*z+x*w)),dim=-1)
        az=(axis*normal).sum(-1,keepdim=True)
        radial=normal-az*axis
        f=((az[...,0]+1)*256).clamp(0,512)
        i=f.long().clamp_max(511)
        p=self.profile[self.wheels,i].lerp(self.profile[self.wheels,i+1],(f-i)[...,None])
        arm=p[...,0:1]*axis-p[...,1:2]*radial/radial.norm(dim=-1,keepdim=True).clamp_min(.01)
        height=((position+arm-ground)*normal).sum(-1)
        contact_velocity=lin+torch.cross(ang,arm,dim=-1)
        lateral=axis-az*normal
        slip=(contact_velocity*lateral).sum(-1).abs()/lateral.norm(dim=-1).clamp_min(.01)
        normal_velocity=(contact_velocity*normal).sum(-1)
        load=torch.where((force>=CONTACT_ON)&active,force,0.)
        slip_cost=(load*(slip/.30).square().clamp(max=1.)).sum(1)/load.sum(1).clamp_min(1.)

        self.off.masked_fill_(changed,0)
        self.on.masked_fill_(changed,0)
        self.air.masked_fill_(changed,0)
        self.peak.masked_fill_(changed,0.)
        self.gap.masked_fill_(changed,0.)
        self.armed.masked_fill_(changed,False)
        self.seen.masked_fill_(changed,False)
        self.off.copy_(torch.where((force<=CONTACT_OFF)&eligible,self.off+1,0))
        self.on.copy_(torch.where((force>=CONTACT_ON)&eligible,self.on+1,0))
        self.armed.logical_or_((self.on>=ON_FRAMES)&(self.air==0)&eligible)
        waiting=self.armed&(self.off>0)
        self.peak.copy_(torch.where(waiting&(self.off==1),height,self.peak))
        self.peak.copy_(torch.where(waiting|(self.air>0),torch.maximum(height,self.peak),self.peak))
        start=waiting&(self.off==OFF_FRAMES)
        self.air.copy_(torch.where(start,OFF_FRAMES,torch.where(self.air>0,self.air+1,0)))
        self.armed.logical_and_(~start)
        landed=(self.air>0)&(self.on==ON_FRAMES)
        duration=(self.air-ON_FRAMES)*dt
        self.gap.add_(eligible.float()*dt)
        valid=landed&(duration>=params["min_s"]-.001)&(duration<=params["max_s"]+.001)
        valid&=(self.peak>=params["min_m"])&(self.peak<=params["max_m"])
        credited=valid&(~self.seen|(self.gap>=params["min_interval_s"]))
        self.seen.logical_or_(credited)
        self.gap.masked_fill_(credited,0.)
        self.armed.logical_or_(landed)
        self.air.masked_fill_(landed,0)
        self.peak.masked_fill_(landed,0.)
        overdue=(((self.gap-params["gap_budget_s"])/.35).clamp(0.,1.)*eligible).sum(1)
        overdue/=eligible.sum(1).clamp_min(1)
        return credited.float().mean(1),overdue,slip_cost,valid.float().sum(1),landed,normal_velocity
