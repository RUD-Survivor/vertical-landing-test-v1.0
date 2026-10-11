"""Measure main-ray uniformity from archived GPU HDR, without modifying images.

Lengths are threshold-dependent image features, not physical ray endpoints.
The common annulus avoids the saturated core; no acceptance threshold is imposed.
"""
import argparse
import json
from pathlib import Path
import numpy as np

ROOT = Path(__file__).resolve().parents[1]

def sample(a, x, y):
    ix, iy = np.floor(x).astype(int), np.floor(y).astype(int)
    tx, ty = x-ix, y-iy
    return ((1-tx)*(1-ty)*a[iy, ix]+tx*(1-ty)*a[iy, ix+1]
            +(1-tx)*ty*a[iy+1, ix]+tx*ty*a[iy+1, ix+1])

def measure(stage):
    folder = ROOT/'build/tests/solar-chain'/f'stage{stage}'
    rgb = np.fromfile(folder/'solar-response-41.bin', dtype='<f2').reshape(720,1280,4)[:,:,:3].astype(float)
    hdr = rgb@np.array([.2126,.7152,.0722])
    angles = np.arange(1440)*2*np.pi/1440
    radii = np.arange(25,281,dtype=float)
    values = sample(hdr,639.5+radii[:,None]*np.cos(angles),359.5+radii[:,None]*np.sin(angles))
    # Same angular smoothing as the independent aperture-count validation.
    values = sum(np.roll(values,i,axis=1) for i in range(-15,16))/31
    profile = values[:21:2].mean(axis=0)
    peaks = np.flatnonzero((profile>np.roll(profile,1))&(profile>=np.roll(profile,-1))
                          &(profile>np.median(profile)+.1*(profile.max()-np.median(profile))))
    strengths = profile[peaks]-np.median(profile)
    # Subtract radial background so wide glare does not masquerade as a ray.
    rays = values[:,peaks]-np.median(values,axis=1)[:,None]
    threshold = float(strengths.max()*.1)
    def lengths_at(fraction,relative=False):
        limits=strengths*fraction if relative else np.full(len(peaks),strengths.max()*fraction)
        return [float(radii[np.flatnonzero(rays[:,i]>limits[i])[-1]])
                if np.any(rays[:,i]>limits[i]) else 0. for i in range(len(peaks))]
    lengths=lengths_at(.1)
    widths={}
    # A local transverse scan: measure half-height above the angular background,
    # rather than counting side lobes or treating the halo as part of the ray.
    offsets=np.linspace(-np.radians(10),np.radians(10),161)
    for radius in [35,60,90]:
        transverse=sample(hdr,639.5+radius*np.cos(angles[peaks,None]+offsets),
                           359.5+radius*np.sin(angles[peaks,None]+offsets))
        transverse-=np.median(values[int(radius-25)])
        measured=[]
        for row in transverse:
            peak=int(row.argmax());half=row[peak]*.5
            left=peak;right=peak
            while left>0 and row[left-1]>=half:left-=1
            while right<len(row)-1 and row[right+1]>=half:right+=1
            if row[peak]<=0 or left==0 or right==len(row)-1:measured.append(None);continue
            l=left-(row[left]-half)/max(row[left]-row[left-1],1e-20)
            r=right+(row[right]-half)/max(row[right]-row[right+1],1e-20)
            measured.append(float(radius*(r-l)*(offsets[1]-offsets[0])))
        widths[str(radius)]=measured
    slopes=[]
    for column in rays.T:
        mask=(radii>=40)&(radii<=110)&(column>strengths.max()*.02)
        slopes.append(float(np.polyfit(np.log(radii[mask]),np.log(column[mask]),1)[0]) if mask.sum()>5 else None)
    result = {'stage':stage,'rays':len(peaks),'angles_degrees':np.degrees(angles[peaks]).tolist(),
              'annular_hdr_strength':strengths.tolist(),
              'strength_coefficient_of_variation':float(strengths.std()/strengths.mean()),
              'strength_max_over_min':float(strengths.max()/strengths.min()),
              'common_hdr_threshold':threshold,'threshold_radius_pixels':lengths,
              'radius_max_over_min':float(max(lengths)/min(lengths)) if min(lengths)>0 else None,
              'common_threshold_radii':{str(f):lengths_at(f) for f in [.02,.05,.1,.2]},
              'per_ray_relative_threshold_radii':{str(f):lengths_at(f,True) for f in [.1,.3,.5]},
              'transverse_fwhm_pixels':widths,'log_radial_decay_slopes_40_110px':slopes,
              'limits':'25-45px annulus; 25-280px extent; 7.75 degree angular smoothing for radial traces. Width uses unsmoothed local scans; null means unresolved/broader than the 20 degree window. No energy or ray-count acceptance.'}
    (folder/'ray-diagnostics.json').write_text(json.dumps(result,indent=2),encoding='utf-8')
    print('Shape diagnostics:',stage,'strength CV',result['strength_coefficient_of_variation'],
          'threshold radius ratio',result['radius_max_over_min'],'widths',widths,'decay slopes',slopes)

if __name__=='__main__':
    parser=argparse.ArgumentParser();parser.add_argument('stage',type=int)
    measure(parser.parse_args().stage)
