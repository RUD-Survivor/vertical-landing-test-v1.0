"""Camera-angle shape distributions from NASA JPEG and actual GPU captures.

No ray-count or energy acceptance. JPEG profiles describe appearance only;
window artifacts, clipping and postprocessing prevent lens metrology.
"""
import argparse
import json
from pathlib import Path
import numpy as np
from solar_chain_compare import polar_samples

ROOT=Path(__file__).resolve().parents[1]/'build/tests'

def shape(path,nasa=False):
    rgb,valid,radii,center=polar_samples(path,nasa,max_degrees=10)
    code=rgb@np.array([.2126,.7152,.0722])
    # Separate broad background from directional features at every radius.
    background=np.percentile(code,20,axis=1)
    excess=np.maximum(code-background[:,None],0)
    excess=sum(np.roll(excess,i,axis=1) for i in range(-2,3))/5
    annulus=(radii>=2)&(radii<=4)
    profile=excess[annulus].mean(axis=0)
    candidates=np.flatnonzero((profile>np.roll(profile,1))&(profile>=np.roll(profile,-1))
                             &(profile>profile.max()*.15))
    # Suppress neighboring fringes; this is feature extraction, not a target
    # number of blades or spikes. No expected count is used.
    peaks=[]
    for index in sorted(candidates,key=lambda i:profile[i],reverse=True):
        if all(min(abs(index-j),720-abs(index-j))>=24 for j in peaks):peaks.append(int(index))
    peaks.sort()
    records=[]
    for index in peaks:
        trace=excess[:,index]
        record={'angle_degrees':index*.5,'annular_code_excess':float(profile[index]),'length_degrees':{},'length_censored':{},'fwhm_degrees':{}}
        for fraction in [.1,.3,.5]:
            active=np.flatnonzero((trace>profile[index]*fraction)&valid[:,index])
            record['length_degrees'][str(fraction)]=float(radii[active[-1]]) if len(active) else None
            record['length_censored'][str(fraction)]=bool(len(active) and (active[-1]==len(radii)-1 or not valid[min(active[-1]+1,len(radii)-1),index]))
        for radius in [2.5,4,6]:
            row=excess[np.argmin(abs(radii-radius))]
            offsets=np.arange(-20,21);window=row[(index+offsets)%720]
            local=int(window.argmax());half=window[local]*.5
            left=local;right=local
            while left>0 and window[left-1]>=half:left-=1
            while right<40 and window[right+1]>=half:right+=1
            width=None
            if window[local]>0 and left>0 and right<40:
                l=left-(window[left]-half)/max(window[left]-window[left-1],1e-20)
                r=right+(window[right]-half)/max(window[right]-window[right+1],1e-20)
                width=float((r-l)*.5)
            record['fwhm_degrees'][str(radius)]=width
        records.append(record)
    def spread(values):
        values=np.array([v for v in values if v is not None],dtype=float)
        return {'p10':float(np.percentile(values,10)),'median':float(np.median(values)),
                'p90':float(np.percentile(values,90)),'cv':float(values.std()/values.mean())} if len(values) else None
    return {'center_pixels':center,'features':records,
            'width_distributions':{str(r):spread([p['fwhm_degrees'][str(r)] for p in records]) for r in [2.5,4,6]},
            'length_distributions':{str(f):spread([p['length_degrees'][str(f)] for p in records]) for f in [.1,.3,.5]},
            'contrast_distribution':spread([p['annular_code_excess'] for p in records])}

def main(stage):
    folder=ROOT/'solar-chain'/f'stage{stage}'
    result={'nasa':shape(ROOT/'solar-reference/ISS067-E-242236.JPG',True),
            'render':shape(folder/'nasa-framing.png'),
            'limits':'Processed JPEG, nominal 35mm full-frame; widths above a 20-degree local window are null, lengths can be support/threshold-limited. Feature distributions are not energy or ray-count acceptance.'}
    (folder/'shape-comparison.json').write_text(json.dumps(result,indent=2),encoding='utf-8')
    for label in ['nasa','render']:
        print(label,'widths',result[label]['width_distributions'],'lengths',result[label]['length_distributions'])

if __name__=='__main__':
    parser=argparse.ArgumentParser();parser.add_argument('stage',type=int);main(parser.parse_args().stage)
