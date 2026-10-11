"""Bake a stable, multi-wavelength aperture PSF; no temporal noise or sprites.

Fraunhofer pupil FFT, nominal 35 mm f/22. Fixed curved blades, transmission
and wavefront are illustrative lens assumptions, not a Nikon prescription.
Output is linear RGB density per radian squared, separately normalized.
"""
import json
from pathlib import Path
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
SIZE = 1024
RADIUS = 300
FFT_EXTENT = .00055 * 22 / 35 * RADIUS
EXTENT = .20
PUPIL_OVERSAMPLE = 4

def sample(a, x, y):
    ix, iy = np.floor(x).astype(int), np.floor(y).astype(int)
    tx, ty = x-ix, y-iy
    h,w=a.shape
    valid = (ix>=0)&(iy>=0)&(ix<w-1)&(iy<h-1)
    ix, iy = np.clip(ix,0,w-2), np.clip(iy,0,h-2)
    return valid*((1-tx)*(1-ty)*a[iy,ix]+tx*(1-ty)*a[iy,ix+1]
                  +(1-tx)*ty*a[iy+1,ix]+tx*ty*a[iy+1,ix+1])

def bake(blades, wavefront_strength=1, wing_split=True, oversample=PUPIL_OVERSAMPLE, ideal=False):
    # Keep physical pupil/window size and output angular spacing fixed. A finer
    # pupil lattice moves its Nyquist replicas outside the stored angular field.
    # Cropping a high-resolution FFT is not scaling the physical aperture.
    work_size=SIZE*oversample;center=work_size/2
    coords=(np.arange(work_size)-center)/(RADIUS*oversample)
    u,v=coords[None,:],coords[:,None]
    if ideal:
        theta=np.arctan2(v,u)-np.pi/18
        sector=(theta+np.pi/blades)%(2*np.pi/blades)-np.pi/blades
        pupil=np.hypot(u,v)*np.cos(sector)<=np.cos(np.pi/blades)
        amplitude=pupil.astype(float)
    else:
        # Intersection of convex curved blade constraints. Parameters belong
        # to one stationary lens, not individually drawn/animated image rays.
        pupil=np.ones((work_size,work_size),dtype=bool)
        for blade in range(blades):
            a=2*np.pi*blade/blades+np.pi/18
            normal=a+.035*np.sin(2*a+.4)
            tangent=-u*np.sin(normal)+v*np.cos(normal)
            offset=np.cos(np.pi/blades)*(1+.045*np.cos(3*a+.7))
            curvature=.075+.065*np.sin(2*a-.3)
            pupil&=(u*np.cos(normal)+v*np.sin(normal)+curvature*tangent*tangent<=offset)
        amplitude=pupil*np.clip(.78-.24*u+.11*v-.18*(.6*u+.8*v)**2,.12,1)
    # Wavefront in micrometres, fixed in lens coordinates across all frames.
    wavefront=wavefront_strength*(.15*(u*u-v*v)+.18*u*(u*u+v*v)
                                  +.07*(u*u*u-3*u*v*v)+.04*(u*u+v*v)**2)
    y,x=np.mgrid[:SIZE,:SIZE]
    rgb=[]
    for wavelength in [.650,.550,.460]:
        field=amplitude*np.exp(2j*np.pi*wavefront/wavelength)
        psf=np.abs(np.fft.fftshift(np.fft.fft2(field)))**2
        scale=wavelength/.550
        # Integrate output cells when storing a wider angular field. A single
        # sample misses thin far-field fringes and creates alias-coloured tips.
        integrated=np.zeros((SIZE,SIZE))
        for sy in range(4):
            for sx in range(4):
                integrated+=sample(psf,(x+(sx+.5)/4-SIZE/2)*EXTENT/FFT_EXTENT/scale+center,
                                       (y+(sy+.5)/4-SIZE/2)*EXTENT/FFT_EXTENT/scale+center)/16
        psf=integrated
        # The direct photosphere already carries the central image. Bake only
        # diffraction wings into the redistribution layer, otherwise its huge
        # central lobe hides all fine structure beneath an extra soft core.
        angular_radius=np.hypot(x+.5-SIZE/2,y+.5-SIZE/2)*2*EXTENT/SIZE
        transition=np.clip((angular_radius/scale-.0006)/.0014,0,1)
        if wing_split:psf*=transition*transition*(3-2*transition)
        if not ideal and wing_split:
            # Weak fixed anisotropic scatter, separate from coherent diffraction.
            # This positive Moffat mixture is a phenomenological surface/window
            # response, not an inferred microstructure or a new ray sprite.
            ax=(x+.5-SIZE/2)*2*EXTENT/SIZE
            ay=(y+.5-SIZE/2)*2*EXTENT/SIZE
            along=.94*ax+.342*ay;across=-.342*ax+.94*ay
            scatter=(1+(along/.018)**2+(across/.0035)**2)**-1.8
            scatter*=transition*transition*(3-2*transition)
            scatter/=scatter.sum()
            psf=.965*psf/psf.sum()+.035*scatter
        # Smooth the finite stored far-field tail rather than cutting visible
        # rays at the old six-degree border. This is a support approximation.
        tail=np.clip((angular_radius-.12)/(.20-.12),0,1)
        psf*=1-tail*tail*(3-2*tail)
        # Guard band prevents a hard border and forbids wraparound tails.
        edge=np.maximum(abs(x+.5-SIZE/2),abs(y+.5-SIZE/2))/(SIZE/2)
        psf*=np.clip((1-edge)/.04,0,1)**2
        psf[[0,-1],:]=0;psf[:,[0,-1]]=0
        psf/=psf.sum()*(2*EXTENT/SIZE)**2
        rgb.append(psf)
    result=np.stack(rgb+[np.zeros_like(rgb[0])],axis=-1).astype('<f4')
    assert np.isfinite(result).all() and result.min()>=0
    sums=result[:,:,:3].sum(axis=(0,1),dtype=np.float64)*(2*EXTENT/SIZE)**2
    print(f'BAKED aperture={blades} finite response peak={result[:,:,:3].max():.6g}')
    return result,sums

if __name__=='__main__':
    out=ROOT/'assets/data/solar';out.mkdir(parents=True,exist_ok=True)
    planes=[];report={'format_version':2,'size':SIZE,'pupil_oversample':PUPIL_OVERSAMPLE,'angular_half_extent':EXTENT,'focal_mm':35,'f_number':22,
                     'cell_quadrature':16,'tail_window_rad':[.12,.20],
                     'lens_profile_revision':3,
                     'assumptions':'Fixed convex curved blades, spatial transmission, illustrative higher-order wavefront and 3.5% anisotropic wing mixture; field-dependent positive response integration at runtime. No measured lens prescription.', 'apertures':{}}
    for blades in [7,8,9]:
        plane,sums=bake(blades);planes.append(plane)
        report['apertures'][str(blades)]={'integrals':sums.tolist()}
    atlas=np.concatenate(planes,axis=0)
    with (out/'aperture-response.rgba32f').open('wb') as f:
        np.array([0x53505347,SIZE,SIZE*3],dtype='<u4').tofile(f)
        np.array([EXTENT],dtype='<f4').tofile(f)
        atlas.tofile(f)
    (out/'aperture-response.json').write_text(json.dumps(report,indent=2),encoding='utf-8')
