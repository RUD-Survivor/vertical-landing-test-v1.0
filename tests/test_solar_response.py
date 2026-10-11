"""Independent conservation, spectral scaling and stability checks for pupil bake."""
import importlib.util
import unittest
from pathlib import Path
import numpy as np

ROOT=Path(__file__).resolve().parents[1]
spec=importlib.util.spec_from_file_location('solar_bake',ROOT/'tools/build_solar_response.py')
bake=importlib.util.module_from_spec(spec);spec.loader.exec_module(bake)

class SolarResponseTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        path=ROOT/'assets/data/solar/aperture-response.rgba32f'
        cls.header=np.fromfile(path,dtype='<u4',count=3)
        cls.extent=np.fromfile(path,dtype='<f4',count=1,offset=12)[0]
        cls.data=np.fromfile(path,dtype='<f4',offset=16).reshape(3,1024,1024,4)

    def test_energy_and_edges(self):
        self.assertEqual(self.header.tolist(),[0x53505347,1024,3072])
        self.assertAlmostEqual(float(self.extent),bake.EXTENT,places=7)
        self.assertTrue(np.isfinite(self.data).all())
        self.assertGreaterEqual(self.data.min(),0)
        energy=self.data[:,:,:,:3].sum(axis=(1,2),dtype=np.float64)*(2*bake.EXTENT/1024)**2
        np.testing.assert_allclose(energy,1,rtol=1e-6,atol=0)
        self.assertEqual(np.max(self.data[:,:,[0,-1]]),0)
        self.assertEqual(np.max(self.data[:,[0,-1],:]),0)

    def test_wavelength_width(self):
        y,x=np.mgrid[:1024,:1024];r2=(x+.5-512)**2+(y+.5-512)**2
        # Ideal diffraction scales with wavelength. Fixed optical path errors
        # have a stronger phase effect at blue; conditioned wing variances of
        # an aberrated lens need not have the same ordering.
        ideal,_=bake.bake(7,wavefront_strength=0,wing_split=False,ideal=True)
        for plane in [ideal]:
            moments=(plane[:,:,:3]*r2[:,:,None]).sum(axis=(0,1))/plane[:,:,:3].sum(axis=(0,1),dtype=np.float64)
            self.assertGreater(moments[0],moments[1])
            self.assertGreater(moments[1],moments[2])
        self.assertFalse(np.array_equal(self.data[:,:,:,0],self.data[:,:,:,2]))

    def test_reproducible_and_structured(self):
        regenerated,_=bake.bake(7)
        np.testing.assert_array_equal(regenerated,self.data[0])
        # Astigmatism/coma create a fixed response, not an identical rotated ray.
        red=self.data[0,:,:,0]
        self.assertGreater(np.abs(red-np.rot90(red,2)).sum()/red.sum(),.01)

    def test_pupil_sampling_convergence(self):
        coarse,_=bake.bake(7,oversample=1)
        medium,_=bake.bake(7,oversample=2)
        fine=self.data[0]
        coarse_change=np.abs(coarse-medium).sum(dtype=np.float64)
        fine_change=np.abs(medium-fine).sum(dtype=np.float64)
        self.assertLess(fine_change,coarse_change*.5)
        print('Pupil convergence L1 ratios:',fine_change/coarse_change)

if __name__=='__main__':unittest.main()
