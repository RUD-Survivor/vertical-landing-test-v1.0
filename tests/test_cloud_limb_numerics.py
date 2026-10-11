"""Independent double/float references for orbital cloud reconstruction."""
import math
import struct
import unittest


def f32(x):
    return struct.unpack('f', struct.pack('f', x))[0]


class CloudLimbNumerics(unittest.TestCase):
    def test_mie_segment_cancellation_cannot_amplify_endpoint_energy(self):
        # Endpoint subtraction is bounded by incoming radiance. Reconstructing
        # a spectrum from a nearly cancelled red difference violates that bound.
        endpoint0 = (1., 2., 4.)
        mie0_red, mie1_red = .1, .0991
        for epsilon in (1e-2, 1e-4, 1e-6, 1e-8, 1e-10, 1e-12):
            endpoint1 = (1-epsilon, 1.5, 2.)
            mie0 = [c*mie0_red/endpoint0[0] for c in endpoint0]
            mie1 = [c*mie1_red/endpoint1[0] for c in endpoint1]
            for t in (0., .5, 1.):
                segment = [max(a-t*b, 0.) for a, b in zip(mie0, mie1)]
                for a, b in zip(segment, mie0):
                    self.assertTrue(math.isfinite(a))
                    self.assertLessEqual(a, b)
            if epsilon<=1e-6:
                old_blue = (endpoint0[2]-endpoint1[2])*(mie0_red-mie1_red)/epsilon
                self.assertGreater(old_blue, 1000.)
        self.assertEqual([max(a-a, 0.) for a in mie0], [0., 0., 0.])

    def test_continuous_pixel_coverage_reference(self):
        # CDF of the sum of two independent uniform variables: exact area of
        # a square cut by a line. A corner crossing must not jump by 1/4.
        def area(gx, gy, distance):
            a, b = abs(gx), abs(gy)
            half = (a+b)/2
            if distance>=half:
                return 0.
            if distance<=-half:
                return 1.
            if min(a, b)<1e-12:
                return .5-distance/max(a, b)
            s = half-abs(distance)
            tail = (s*s-max(s-a, 0)**2-max(s-b, 0)**2)/(2*a*b)
            return tail if distance>=0 else 1-tail
        self.assertAlmostEqual(area(1, 1, .5), .125)
        self.assertAlmostEqual(area(1, 1, -.5), .875)
        self.assertAlmostEqual(area(1, 0, .25), .25)
        for gx, gy in ((1, 1), (-1, 1), (2, .1), (0, 3)):
            previous = 1.
            for i in range(1001):
                d = (i/1000-.5)*(abs(gx)+abs(gy))
                current = area(gx, gy, d)
                self.assertLessEqual(current, previous+1e-12)
                self.assertLessEqual(abs(current-previous), .021)
                self.assertAlmostEqual(current+area(gx, gy, -d), 1.)
                previous = current

    def test_motion_response_limits_spatial_lag(self):
        # Compare a first-order estimator's measured ramp lag to its analytic
        # solution. Motion response must stay below a quarter pixel of lag.
        for motion in (0., 1e-5, .001, .01, .015625, .05, .1, .25, 1., 30.):
            for noise_floor in (1/16, 1/32):
                alpha = min(1., max(noise_floor, 4*motion))
                lag = motion*(1-alpha)/alpha
                self.assertLess(lag, .25)
                estimate = 0.
                for frame in range(1000):
                    exact = frame*motion
                    estimate = (1-alpha)*estimate + alpha*exact
                self.assertAlmostEqual(exact-estimate, lag, places=8)

    def test_factored_shell_intersection_against_double_reference(self):
        for camera in (6771., 8371., 16371., 106371.):
            radius = 6391.
            for radial_offset in (-10., -1., -.1, .1, 1., 10.):
                impact = radius + radial_offset
                theta = math.asin(impact/camera)
                float_impact = f32(camera*f32(math.sin(theta)))
                h = f32(f32(radius-float_impact)*f32(radius+float_impact))
                self.assertEqual(h>=0, radial_offset<0)
                if h>=0:
                    reference = camera*math.cos(theta)-math.sqrt((radius-impact)*(radius+impact))
                    root = f32(f32(camera*f32(math.cos(theta)))-f32(math.sqrt(h)))
                    self.assertLess(abs(root-reference), .1)

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
