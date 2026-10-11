"""Propose camera-profile coefficients from angular JPEG statistics.

Uses actual GPU diffraction residual, fixed exposure, positive scatter, and an
explicit <8% redistribution bound. This is appearance fitting, not lens metrology.
Prints a proposal only; does not modify source or replace GPU validation.
"""
import json
import math
import sys
import numpy as np
from solar_chain_compare import ROOT,interpolate,polar_samples

reference,_,radii,_=polar_samples(ROOT/'solar-reference/ISS067-E-242236.JPG',True)
reference=reference[::2,::4];radii=radii[::2]
angles=np.linspace(0,2*np.pi,180,endpoint=False)
fy=35/12;aspect=1.5;fx=fy/aspect
source=np.array([.51/fx,-.21/fy,1]);source/=np.linalg.norm(source)
east=np.cross([0,1,0],source);east/=np.linalg.norm(east);north=np.cross(source,east)
t=np.radians(radii)[:,None,None];p=angles[None,:,None]
rays=np.cos(t)*source+np.sin(t)*(np.cos(p)*east+np.sin(p)*north)
ndc=rays[:,:,:2]/rays[:,:,2,None]*[fx,fy]
x=(ndc[:,:,0]+1)*600-.5;y=(ndc[:,:,1]+1)*400-.5
folder=ROOT/'solar-chain'/f'stage{int(sys.argv[1]) if len(sys.argv)>1 else 4}'
body=interpolate(np.fromfile(folder/'solar-body-44.bin',dtype='<f2').reshape(800,1200,4)[:,:,:3].astype(float),x,y)[0]
final=interpolate(np.fromfile(folder/'solar-final-44.bin',dtype='<f2').reshape(800,1200,4)[:,:,:3].astype(float),x,y)[0]
flux=1.1*fx*fy/source[2]**3
radius=fy*696340/149597870.7/source[2]**2
tints=np.array([[1,.98,.95],[1,.98,.95],[.64,.84,1],[.58,.80,1],[.56,.76,1]])
dispersion=np.array([1,.97,.94])

def units(sigmas):
    width=np.hypot(fy*np.array(sigmas),math.sqrt(.225)*radius)[:,None,None,None]*dispersion
    relative=(ndc-[.51,-.21])*[aspect,1]
    q=(relative**2).sum(axis=2)[None,:,:,None]
    response=flux*aspect/(2*math.pi*width**2)*np.exp(-q/(2*width**2))*tints[:,None,None,:]
    return response.astype(np.float32)

profile=json.loads((ROOT/'solar-camera-profile.json').read_text())
old=units([k[0] for k in profile['kernels']])
old_fractions=np.array([k[2] for k in profile['kernels']])
star=np.maximum(0,final-body-(old*old_fractions[:,None,None,None]).sum(axis=0)).astype(np.float32)

def display(hdr):
    v=np.maximum(0,hdr*3.25)
    v=np.clip(v*(2.51*v+.03)/(v*(2.43*v+.59)+.14)*(2.43/2.51),0,1)
    return np.where(v<=.0031308,v*12.92,1.055*v**(1/2.4)-.055)

rows=[(radii>=lo)&(radii<hi) for lo,hi in [(0.5,1),(1,2),(2,4),(4,6)]]
def stats(rgb):
    code=rgb@[.2126,.7152,.0722];features=[]
    for row in rows:
        angular=np.median(code[row],axis=0)
        features.extend([np.median(code[row]),*(rgb[row].mean(axis=(0,1))),np.percentile(angular,95)-np.percentile(angular,5)])
    area=(((rgb.min(axis=2)>.985)&(radii[:,None]<=3))*radii[:,None]).sum()/180*(radii[1]-radii[0])*2
    features.append(math.sqrt(area))
    return np.array(features)
target=stats(reference);weights=np.tile([3,1,1,1,2],4).tolist()+[.25]
weights=np.array(weights)
rng=np.random.default_rng(20261011);best=None
for middle_sigma in [.012,.016,.020,.024]:
    u=units([.0015,.006,middle_sigma,.060,.15])
    def evaluate(q):
        fractions=np.exp(q[:5]);star_scale=np.exp(q[5]);total=fractions.sum()+star_scale*profile['star_fraction']+sum(g[1] for g in profile['ghosts'])
        if total>.08:return 1e3,None
        hdr=body*(1-total)/(1-profile['scattered_fraction'])+(u*fractions[:,None,None,None]).sum(axis=0)+star*star_scale
        measured=stats(display(hdr))
        return float(np.mean(((measured-target)*weights)**2)),measured
    q=np.log(np.r_[old_fractions,1.]);error,_=evaluate(q)
    for step in np.geomspace(.8,.015,12):
        for iteration in range(22):
            # Saturated JPEG core cannot identify compact-scatter energy.
            # Keep those two coefficients fixed instead of fitting white pixels.
            trial=q.copy();j=int(rng.integers(2,6));trial[j]+=rng.uniform(-step,step)
            candidate,_=evaluate(trial)
            if candidate<error:q,error=trial,candidate
    _,measured=evaluate(q)
    record={'sigma_middle':middle_sigma,'fractions':np.exp(q[:5]).tolist(),'star_fraction':float(np.exp(q[5])*profile['star_fraction']),
            'objective':error,'target_features':target.tolist(),'predicted_features':measured.tolist(),
            'assumptions':'JPEG appearance; nominal full frame 35mm; fixed exposure 3.25; fitted window/lens combined scatter.'}
    print(json.dumps(record),flush=True)
    if best is None or error<best['objective']:best=record
(ROOT/'solar-chain/profile-proposal.json').write_text(json.dumps(best,indent=2),encoding='utf-8')
