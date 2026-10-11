"""Independent positive FP16 blend precision envelope for the faint solar case."""
import math
import json
from pathlib import Path
import numpy as np
from PIL import Image

root = Path(__file__).resolve().parents[1] / 'build/tests'
profile=json.loads((root/'solar-camera-profile.json').read_text())
w, h = 1280, 720
fy = 1 / math.tan(.4)
aspect = w / h
s = 696340 / (149597870.7 * 100)
omega = 2 * math.pi * s*s / (1 + math.sqrt(1-s*s))
radiance = float(np.float32(1.1)) / (math.pi * (696340/149597870.7)**2)
flux = radiance * omega * fy*fy / aspect
body = np.fromfile(root/'solar-body-33.bin', dtype='<f2').reshape(h,w,4)[:,:,0].astype(float)
grid = (np.arange(32)+.5)/32*10-5
xx, yy = np.meshgrid(w/2-.5+grid, h/2-.5+grid)
ix, iy = np.floor(xx).astype(int), np.floor(yy).astype(int)
tx, ty = xx-ix, yy-iy
samples = ((1-tx)*(1-ty)*body[iy,ix] + tx*(1-ty)*body[iy,ix+1]
           + (1-tx)*ty*body[iy+1,ix] + tx*ty*body[iy+1,ix+1])
transmission = min(1, samples.mean()*100*4/(w*h)/(flux*(1-profile['scattered_fraction'])))

def half(value, truncate):
    result = value.astype('<f2')
    if truncate:
        bits = result.view('<u2').copy()
        bits[result.astype(float)>value] -= 1
        result = bits.view('<f2')
    return result.astype(float)

models = [np.zeros((h,w)), np.zeros((h,w))]
for angular_x, angular_y, fraction, tint in profile['kernels']:
    def mass(n, angular):
        sigma = math.hypot(fy*angular, math.sqrt(.225)*fy*s/math.sqrt(1-s*s))
        pixels = sigma*h/2
        x = np.arange(n)+.5-n/2
        erf = lambda a: np.array([math.erf(v) for v in a])
        return .5*(erf((x+.5)/(math.sqrt(2)*pixels))-erf((x-.5)/(math.sqrt(2)*pixels))) * (abs(x)<=4*pixels)
    value = np.outer(mass(h,angular_y),mass(w,angular_x))*flux*fraction*tint*transmission*w*h/4
    for index in range(2):
        models[index] = half(models[index]+half(value,index==1),index==1)
# Fixed positive binomial defocus convolution of the seven-leaf aperture.
# On-axis faint-source fixture; the lens profile has channel-dependent area.
for radius,fraction,tint in profile['ghosts']:
    ndc_radius=math.hypot(fy*radius,math.sqrt(.225)*fy*s/math.sqrt(1-s*s))
    extent_ghost=int(math.ceil(ndc_radius*h/2*1.2))
    gx,gy=np.meshgrid(np.arange(-extent_ghost,extent_ghost)+.5,np.arange(-extent_ghost,extent_ghost)+.5)
    local_x=gx*2/h/ndc_radius;local_y=gy*2/h/ndc_radius
    value=np.zeros_like(gx)
    for yy in range(4):
        for xx in range(4):
            for by in range(-1,2):
                for bx in range(-1,2):
                    px=(local_x+((xx+.5)/4-.5)*2/h/ndc_radius+bx*.07)/1.015
                    py=(local_y+((yy+.5)/4-.5)*2/h/ndc_radius+by*.07)/1.015
                    angle=np.arctan2(py,px)-math.radians(10)
                    sector=(angle+math.pi/7)%(2*math.pi/7)-math.pi/7
                    inside=np.hypot(px,py)*np.cos(sector)<=math.cos(math.pi/7)
                    value+=inside*(2 if bx==0 else 1)*(2 if by==0 else 1)/256/1.015**2
    area=.5*7*math.sin(2*math.pi/7)*ndc_radius**2/aspect
    value*=flux*fraction*tint*transmission/area
    region_ghost=np.s_[h//2-extent_ghost:h//2+extent_ghost,w//2-extent_ghost:w//2+extent_ghost]
    for index in range(2):models[index][region_ghost]=half(models[index][region_ghost]+half(value,index==1),index==1)
# Independently reconstruct the transported source image and sample the
# linear response atlas. All pixel integration is double precision on the CPU.
atlas=np.fromfile(root.parents[1]/'assets/data/solar/aperture-response.rgba32f',dtype='<f4',offset=16).reshape(3,1024,1024,4)[0,:,:,0].astype(float)
half_extent=.20
extent=int(math.ceil((fy*half_extent+10/h)*h/2))
px,py=np.meshgrid(np.arange(w//2-extent,w//2+extent)+.5-w/2,
                  np.arange(h//2-extent,h//2+extent)+.5-h/2)
star=np.zeros_like(px)
grid16=(np.arange(16)+.5)/16*10-5
gx,gy=np.meshgrid(w/2-.5+grid16,h/2-.5+grid16)
ix,iy=np.floor(gx).astype(int),np.floor(gy).astype(int)
tx,ty=gx-ix,gy-iy
source16=(1-tx)*(1-ty)*body[iy,ix]+tx*(1-ty)*body[iy,ix+1]+(1-tx)*ty*body[iy+1,ix]+tx*ty*body[iy+1,ix+1]
for cy in range(16):
    for cx in range(16):
        if source16[cy,cx]==0:continue
        sx,sy=grid16[cx],grid16[cy]
        weight=source16[cy,cx]/source16.sum()*transmission
        for yy in range(2):
            for xx in range(2):
                ax=(px+(xx+.5)/2-.5-sx)*2/h/fy
                ay=(py+(yy+.5)/2-.5-sy)*2/h/fy
                tx=(ax/(2*half_extent)+.5)*1024-.5
                ty=(ay/(2*half_extent)+.5)*1024-.5
                ix,iy=np.floor(tx).astype(int),np.floor(ty).astype(int)
                valid=(ix>=0)&(iy>=0)&(ix<1023)&(iy<1023)
                ix,iy=np.clip(ix,0,1022),np.clip(iy,0,1022)
                dx,dy=tx-ix,ty-iy
                value=(1-dx)*(1-dy)*atlas[iy,ix]+dx*(1-dy)*atlas[iy,ix+1]+(1-dx)*dy*atlas[iy+1,ix]+dx*dy*atlas[iy+1,ix+1]
                star+=valid*value*weight
star*=flux*profile['star_fraction']*aspect/(fy*fy*4)
region=np.s_[h//2-extent:h//2+extent,w//2-extent:w//2+extent]
for index in range(2):
    models[index][region]=half(models[index][region]+half(star,index==1),index==1)
actual = np.fromfile(root/'solar-response-33.bin',dtype='<f2').reshape(h,w,4)[:,:,0].astype(float).sum()*4/(w*h)
predicted = [m.sum()*4/(w*h) for m in models]
print(f'FP16 reference: transmission={transmission:.9f}, actual={actual:.10g}, RTZ={predicted[1]:.10g}, RNE={predicted[0]:.10g}')
assert min(predicted)*.99 <= actual <= max(predicted)*1.01
for name, case in [('solar-1au',0),('solar-half-au',1),('solar-near',6),('solar-100au',5),('solar-partial',9),('solar-off-axis',14),('solar-photosphere-exposure',35),('solar-sky-context',37),('solar-orbit-context',38),('solar-day-context',39),('solar-cloud-context',40),('solar-nasa-framing',44)]:
    Image.open(root/f'solar-optics-{case}.ppm').save(root/f'{name}.png')
