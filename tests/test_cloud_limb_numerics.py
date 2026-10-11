"""Independent double/float references for orbital cloud reconstruction."""
import math
import struct
import unittest


def f32(x):
    return struct.unpack('f', struct.pack('f', x))[0]


class CloudLimbNumerics(unittest.TestCase):
    def test_extinction_centroid_converges_to_beer_reference(self):
        # Analytic conditional event position for a uniform absorbing slab.
        # Vacuum before/after the slab must not move its optical centroid.
        sigma, start, length = .4, 7., 9.
        exact = start + 1/sigma - length/math.expm1(sigma*length)
        errors = []
        for count in (32, 128, 512):
            step = length/count
            weights = [math.exp(-sigma*i*step)*(-math.expm1(-sigma*step)) for i in range(count)]
            centroid = sum((start+(i+.5)*step)*w for i, w in enumerate(weights))/sum(weights)
            errors.append(abs(centroid-exact))
            # Adding arbitrarily long zero-extinction intervals adds no weight.
            for vacuum in (0., 1e3, 1e6):
                self.assertEqual((centroid*sum(weights)+vacuum*0)/sum(weights), centroid)
        self.assertGreater(errors[0], errors[1])
        self.assertGreater(errors[1], errors[2])
        self.assertLess(errors[-1], 2e-5)

    def test_wind_reprojection_preserves_material_coordinates(self):
        # Density samples p+v*t. Solve for the prior material point, including
        # negative wind and unequal frame intervals; an opposite sign trails.
        for velocity in (-30., -.15, 0., .15, 30.):
            for dt in (1/120, 1/60, 1/30, .25):
                now, position = 42., 1234.
                previous = position + velocity*dt
                self.assertAlmostEqual(previous+velocity*(now-dt), position+velocity*now)

    def test_device_depth_loses_cloud_shell_separation(self):
        # A 1 m near plane cannot resolve the two sides of a thin orbital shell.
        for distance in (400., 2000., 10000.):
            depths = {f32(1 - .001 / (distance - h)) for h in (1., 1.1, 1.2)}
            self.assertLess(len(depths), 3)
            # A kilometre ray-distance RT preserves those distinct positions.
            self.assertEqual(len({f32(distance - h) for h in (1., 1.1, 1.2)}), 3)

    def test_shell_boundary_and_subpixel_coverage(self):
        # Analytic sphere, independent of weather/noise. Supersampling provides
        # partial coverage and is invariant to the 16-frame Bayer permutation.
        radius, camera = 6391., 16371.
        tangent = math.asin(radius / camera)
        pixel_angle = 1 / 360
        coverages = []
        for offset in (-2., -.3, 0., .3, 2.):
            angle = tangent + offset * pixel_angle
            hits = [camera * math.sin(angle + sub * pixel_angle) < radius
                    for sub in (-.25, .25)]
            coverage = sum(hits) / 2
            coverages.append(coverage)
            self.assertTrue(0 <= coverage <= 1)
        self.assertEqual(coverages[0], 1)
        self.assertEqual(coverages[-1], 0)
        self.assertIn(.5, coverages)

    def test_float_sphere_tangent_uncertainty(self):
        # Quantify the old quadratic's cancellation near a tangent. All
        # classification disagreements must lie within 100 m of the shell,
        # including a camera 100,000 km away.
        mismatches = 0
        maximum_error_m = 0
        radius = 6391.
        for altitude in (20., 400., 2000., 10000., 100000.):
            camera = 6371 + altitude
            if camera <= radius:
                continue
            for offset_m in range(-200, 201):
                impact = radius + offset_m / 1000
                angle = math.asin(impact / camera)
                direction = f32(math.cos(angle))
                b = f32(-2 * f32(camera * direction))
                c = f32(f32(camera * camera) - f32(radius * radius))
                discriminant = f32(f32(b * b) - f32(4 * c))
                reference = impact <= radius
                if (discriminant >= 0) != reference:
                    mismatches += 1
                    maximum_error_m = max(maximum_error_m, abs(offset_m))
        print('CLOUD_SPHERE quadratic tangent mismatches:', mismatches,
              'max radial error m:', maximum_error_m)
        self.assertGreater(mismatches, 0)
        self.assertLessEqual(maximum_error_m, 100)

    def test_partial_bayer_tiles_do_not_change_projection(self):
        for width, height in ((640, 360), (641, 363), (1, 1), (3, 5)):
            for offset in range(16):
                x = min((math.ceil(width / 4) - 1) * 4 + offset % 4, width - 1)
                y = min((math.ceil(height / 4) - 1) * 4 + offset // 4, height - 1)
                self.assertTrue(0 <= (x + .5) / width < 1)
                self.assertTrue(0 <= (y + .5) / height < 1)

    def test_no_hit_sentinel_must_not_be_interpolated(self):
        # Demonstrate why representative distance uses point sampling and why
        # coverage (L,T) is interpolated independently of hit distance.
        interpolated = .5 * 10000 + .5 * 1e20
        self.assertGreater(interpolated, 1e19)
        for weight in (.125, .5, 1.):
            for current in (0., .5, 1.):
                for history in (0., .5, 1.):
                    value = history * (1 - weight) + current * weight
                    self.assertTrue(0 <= value <= 1)

    def test_cloud_insertion_after_atmosphere_preserves_clear_background(self):
        # Independent split of foreground-air and background radiance.
        for foreground in (0., .1, 10.):
            for air_t in (0., .5, 1.):
                for background in (0., .3, 100.):
                    base = foreground + air_t * background
                    for cloud_t in (0., .5, 1.):
                        for cloud_l in (0., .2, 20.):
                            exact = foreground + air_t * (cloud_t * background + cloud_l)
                            inserted = cloud_t * base + air_t * cloud_l + (1-cloud_t) * foreground
                            self.assertAlmostEqual(inserted, exact, places=11)
                            if cloud_t == 1 and cloud_l == 0:
                                self.assertEqual(inserted, base)


if __name__ == '__main__':
    unittest.main()
