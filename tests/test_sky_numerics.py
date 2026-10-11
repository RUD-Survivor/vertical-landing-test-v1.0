"""Independent references for catalog integrity, flux and camera integration."""
import math
import struct
import unittest
from pathlib import Path


def pixel_mass(x, sigma):
    return .5 * (math.erf((x + .5) / (math.sqrt(2) * sigma)) - math.erf((x - .5) / (math.sqrt(2) * sigma)))


class SkyNumerics(unittest.TestCase):
    def test_catalog(self):
        data = (Path(__file__).resolve().parents[1] / "assets/data/sky-stars.bin").read_bytes()
        magic, count = struct.unpack_from("<II", data)
        self.assertEqual(magic, 0x53544152)
        self.assertEqual(count, 8920)
        self.assertEqual(len(data), 8 + 32 * count)
        last = math.inf
        for record in struct.iter_unpack("<8f", data[8:]):
            self.assertTrue(all(math.isfinite(x) for x in record))
            self.assertAlmostEqual(sum(x * x for x in record[:3]), 1., places=6)
            self.assertGreater(record[3], 0.)
            self.assertLessEqual(record[3], last)
            self.assertAlmostEqual(sum(a * b for a, b in zip(record[4:7], (.2126, .7152, .0722))), 1., places=6)
            last = record[3]

    def test_magnitude_energy_ratio(self):
        self.assertAlmostEqual(10 ** (-.4 * 0) / 10 ** (-.4 * 5), 100.)

    def test_psf_energy_and_subpixel_phase(self):
        for sigma in (.4, .65, 1.3, 2.):
            for phase in (0., .17, .5, .93):
                for halo in (0., .03, .15):
                    core = sum(pixel_mass(i - phase, sigma) for i in range(-40, 41)) ** 2
                    wide = sum(pixel_mass(i - phase, sigma * 2) for i in range(-40, 41)) ** 2
                    self.assertAlmostEqual((1 - halo) * core + halo * wide, 1., places=12)

    def test_psf_finite_support(self):
        for sigma in (.4, .65, 2.):
            radius = math.ceil(8 * sigma + 1)
            mass = sum(pixel_mass(i - .49, sigma * 2) for i in range(-radius, radius + 1)) ** 2
            self.assertGreater(mass, .9999)

    def test_galactic_rotation_orthonormal(self):
        rows = ((-.0548755604, -.8734370902, -.4838350155), (.4941094279, -.4448296300, .7469822445), (-.8676661490, -.1980763734, .4559837762))
        for i in range(3):
            for j in range(3):
                self.assertAlmostEqual(sum(a*b for a, b in zip(rows[i], rows[j])), float(i == j), places=8)

    def test_milky_way_mips(self):
        import numpy as np
        data = (Path(__file__).resolve().parents[1] / 'assets/data/milky-way.bin').read_bytes()
        self.assertEqual(struct.unpack_from('<4I', data), (0x594B574D, 2048, 1024, 12))
        offset, w, h, reference = 16, 2048, 1024, None
        for _ in range(12):
            values = np.frombuffer(data, '<f2', w*h*4, offset).reshape(h,w,4).astype(np.float64)
            self.assertTrue(np.isfinite(values).all())
            self.assertTrue((values >= 0).all())
            self.assertTrue((values[:,:,3] == 1).all())
            weights = np.diff(np.sin(np.linspace(-math.pi/2, math.pi/2, h+1)))
            integral = (values[:,:,:3]*weights[:,None,None]).sum(axis=(0,1))/w
            if reference is None:
                reference = integral
            self.assertLess(float(np.max(np.abs(integral/reference-1))), .001)
            offset += w*h*8
            w, h = max(1,w//2), max(1,h//2)
        self.assertEqual(offset, len(data))

    def test_celestial_seam_derivative(self):
        for ra in (0., .4, math.pi-1e-5, -math.pi+1e-5):
            step=1e-7
            delta=math.remainder(math.atan2(math.sin(ra+step),math.cos(ra+step))-ra,2*math.pi)
            self.assertAlmostEqual(delta/step,1.,places=7)

    def test_pixel_solid_angle_quadrature(self):
        # Integrating the perspective Jacobian should match the rectangle's
        # analytic solid angle, including off-axis pixels and wide FOV.
        for fov in (.4, .8, 1.3):
            ny, nx = 180, 320
            b = math.tan(fov/2); a = b*nx/ny
            measured = sum(4*a*b/(nx*ny)*(1+((2*(x+.5)/nx-1)*a)**2+((2*(y+.5)/ny-1)*b)**2)**-1.5 for y in range(ny) for x in range(nx))
            exact = 4*math.atan(a*b/math.sqrt(1+a*a+b*b))
            self.assertLess(abs(measured/exact-1), 1e-5)


if __name__ == "__main__":
    unittest.main()
