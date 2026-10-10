"""CPU numerical regression for God Ray conventions; run with Python stdlib.

These tests cover projection round trips, coordinate frames, shadow depth,
and heterogeneous front-to-back radiative transfer. They do not execute GPU code.
"""
import math
from pathlib import Path
import struct
import unittest


def f32(x):
    return struct.unpack('f', struct.pack('f', x))[0]


def projected_depth(n, f, axial_km):
    # Mat4::perspective + terrain.vert's clip-Z remapping, float32 coefficients.
    a = f32((f + n) / (n - f))
    b = f32(2 * f * n / (n - f))
    return f32(0.5 * (-a + b / axial_km) + 0.5), a, b


def recovered_distance_m(uv, depth, a, b, fov=0.8, aspect=16 / 9):
    if depth >= 1:
        return 8000.0
    # Analytic inverse of the actual perspective matrix (km), then ray length.
    z = -b / ((2 * depth - 1) + a)
    y = (1 - 2 * uv[1]) * math.tan(fov / 2)
    x = (2 * uv[0] - 1) * math.tan(fov / 2) * aspect
    return abs(z) * math.sqrt(1 + x * x + y * y) * 1000


class GodRayNumerics(unittest.TestCase):
    def test_actual_projection_and_off_axis_distance(self):
        max_error = 0
        for n, far in [(0.00001, 1e7), (0.001, 10000), (0.01, 1e10)]:
            for uv in [(0.5, 0.5), (0.05, 0.1), (0.95, 0.9)]:
                for z in [max(n * 1.1, 0.015), 0.05, 0.2, 0.4]:
                    depth, a, b = projected_depth(n, far, z)
                    x = (2 * uv[0] - 1) * math.tan(0.4) * 16 / 9
                    y = (1 - 2 * uv[1]) * math.tan(0.4)
                    expected = z * math.sqrt(1 + x * x + y * y) * 1000
                    actual = recovered_distance_m(uv, depth, a, b)
                    max_error = max(max_error, abs(actual - expected))
                    self.assertAlmostEqual(actual / expected, 1, delta=0.002)
                    self.assertAlmostEqual(min(actual, 400), min(expected, 400), delta=0.8)
        print(f'Projection: 36 cases, max distance error {max_error:.6f} m')

    def test_sky(self):
        self.assertEqual(recovered_distance_m((0, 0), 1, -1, -0.02), 8000)

    def test_coordinate_frame(self):
        camera = (10, 20, 30)
        relative_camera = (0, 6371000, 0)
        offset = (100, 200, -50)
        relative_sample = tuple(a + b for a, b in zip(relative_camera, offset))
        expected = tuple(a + b * 0.001 for a, b in zip(camera, offset))
        restored = tuple(a + (b - c) * 0.001 for a, b, c in
                         zip(camera, relative_sample, relative_camera))
        self.assertEqual(restored, expected)
        self.assertEqual(expected, (10.1, 20.2, 29.95))

    def test_shadow_depth_and_bias(self):
        for ndc, stored, visible in [(-0.6, 0.5, True), (0.6, 0.5, False),
                                     (0.0, 0.5, True), (0.9, 1.0, True)]:
            receiver = ndc * 0.5 + 0.5 - 0.001
            self.assertEqual(receiver <= stored, visible)

    def test_sample_bounds(self):
        for distance in [0, 0.01, 50, 400]:
            for jitter in [0, 0.5, 1]:
                samples = [(i + jitter) * distance / 64 for i in range(64)]
                self.assertTrue(all(0 <= t <= distance for t in samples))

    def test_heterogeneous_integration(self):
        # Two 200m slabs with different illumination: ordering must matter.
        sigma, length = 0.0002, 200
        t = math.exp(-sigma * length)
        expected = 2 * (1 - t) + t * 10 * (1 - t)
        light, trans = 0, 1
        for i in range(64):
            source = 2 if i < 32 else 10
            step_t = math.exp(-sigma * 400 / 64)
            light += trans * source * (1 - step_t)
            trans *= step_t
        self.assertAlmostEqual(light, expected, places=12)
        self.assertAlmostEqual(trans, math.exp(-0.08), places=12)
        reversed_light = 10 * (1 - t) + t * 2 * (1 - t)
        self.assertGreater(abs(light - reversed_light), 0.01)
        print(f'Integration: L={light:.9f}, T={trans:.9f}; analytic match')

    def test_shader_conventions(self):
        shader = (Path(__file__).resolve().parents[1] /
                  'src/render/shaders/cloud/cloud_composite.glsl').read_text(encoding='utf-8')
        for expression in ['sceneZ * 2.0 - 1.0',
                           'length(sceneViewH.xyz / sceneViewH.w) * 1000.0',
                           'camPosAbsKm.xyz + rayOffsetM * 0.001',
                           'shadowCoord.z * 0.5 + 0.5',
                           'shadowPosOnAltas.z <= depthShadow',
                           'marchingDistance * a * a',
                           'visibilityTerm *= godRayCloudVisibility']:
            self.assertIn(expression, shader)

    def test_quadratic_partition(self):
        for distance in [0, 50, 1000, 8000]:
            steps = max(8, min(48, math.ceil(distance / 125)))
            edges = [distance * (i / steps) ** 2 for i in range(steps + 1)]
            widths = [b - a for a, b in zip(edges, edges[1:])]
            self.assertAlmostEqual(sum(widths), distance)
            self.assertTrue(all(w >= 0 for w in widths))
            for jitter in [0, 0.5, 1]:
                samples = [a + (b - a) * jitter for a, b in zip(edges, edges[1:])]
                self.assertTrue(all(0 <= t <= distance for t in samples))

    def test_adaptive_budget(self):
        for distance, expected in [(400, 8), (1000, 8), (4000, 32), (8000, 48)]:
            self.assertEqual(max(8, min(48, math.ceil(distance / 125))), expected)

    def test_cloud_base_clip(self):
        radius, camera = 6372.0, 6371.0
        for cosine in [0, 0.5, 1]:
            b = camera * cosine
            exit_km = -b + math.sqrt(b * b + radius * radius - camera * camera)
            distance = min(8, exit_km)
            end_radius = math.sqrt(camera * camera + distance * distance + 2 * b * distance)
            self.assertLessEqual(end_radius, radius + 1e-9)

    def test_cloud_gap_contrast(self):
        # Only direct light is attenuated: ambient remains in the cloud shadow.
        sigma, distance, ambient, direct = 0.0002, 1000, 1, 10
        scattered = lambda cloud_t: (ambient + direct * cloud_t) * (1 - math.exp(-sigma * distance))
        clear, under_cloud = scattered(1), scattered(0.05)
        self.assertGreater(clear / under_cloud, 7)
        self.assertGreater(under_cloud, 0)
        print(f'Cloud gap: clear/opaque scattering ratio {clear / under_cloud:.3f}')

    def test_atlas_slice_clamp(self):
        dim = 512
        for c in range(4):
            for uv in [0, 0.001, 0.5, 0.999, 1]:
                p = max(0, min(uv * dim - 0.5, dim - 1))
                lo = math.floor(p)
                hi = min(lo + 1, dim - 1)
                self.assertTrue(c * dim <= c * dim + lo <= c * dim + hi < (c + 1) * dim)


if __name__ == '__main__':
    unittest.main(verbosity=2)
