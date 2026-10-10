# Cloud-gap God Ray implementation and cost

The fog march now multiplies direct sunlight by the current frame's cloud-column
transmittance atlas. Ambient light is retained. Terrain shadowing remains optional.
Enable God Ray and cloud-to-ground shadows in F3. Cloud coverage must leave gaps;
a fully opaque sky cannot produce bright shafts.

The march stops at opaque geometry, cloud base or 8 km, whichever is nearest.
It uses ceil(distance_m / 125) steps clamped to 8–48, a quadratic distance
partition, and frame-varying jitter. Negligible-density samples skip lighting;
transmittance below 0.01 terminates the march. Cameras within/above the cloud
layer skip this below-cloud fog effect. The atlas contains whole-column T and
cannot represent partial cloud occlusion inside the cloud layer.

## Operation budget (estimates, not GPU timings)

Previous march: 64 steps per active full-resolution pixel, up to one terrain
depth fetch and one atmosphere LUT fetch per step; no cloud occlusion.

New march: 8–48 steps; four R32F image loads per cloud lookup (bilinear),
eight at a cascade blend. At most one terrain depth fetch and one atmosphere
LUT fetch per step remain. Cloud shadow disabled/outside all cascades: zero
cloud atlas loads. All figures exclude the pre-existing cloud raymarch and
ambient lookup, and assume every sample survives density/opacity culling.

- 400 m segment: 8 steps, 32 cloud atlas loads, versus 64 old steps.
- 1 km segment: 8 steps, 32 cloud atlas loads.
- 4 km segment: 32 steps, 128 cloud atlas loads.
- 8 km segment: 48 steps, 192 cloud atlas loads (384 at cascade blends).
- 1920×1080 worst case: 99,532,800 fog steps and 398,131,200 atlas loads/frame;
  old march: 132,710,400 steps/frame. This is 25% fewer iterations, not a
  claim of 25% faster rendering: additional cloud lookups add work.
- 3840×2160: four times those operation counts.

With both shadow types enabled, the worst-case per-pixel image access count
rises from 128 to 288 (48×[4 cloud + 1 terrain + 1 atmosphere]), or 480 at
cascade blends. These are shader access counts, not DRAM transactions: cache
reuse matters significantly. No per-pixel cloud-density/light raymarch is added.

The existing default 4×512×512 R32F cloud atlas is 4 MiB. No new GPU images,
buffers or descriptor bindings are allocated. Its existing 48-step generation
budget is 50,331,648 cloudMap evaluations/frame, unchanged when cloud shadows
were already enabled. If previously disabled, enabling cloud shadows also adds
this generation cost. The atlas is generated before composite and kept GENERAL
until fog reads complete, with compute write→read and compute→fragment barriers.

## Verification and limitations

Run `python tests/test_godray_numerics.py`. Tests cover projection, coordinate
frames, shadow depth/bias, integration ordering, quadratic partition bounds,
atlas-slice clamping, and cloud-gap scattering contrast. GLSL compilation and
SPIR-V validation check the shader interface and bytecode. CPU numerical tests
do not measure GPU time or verify the rendered image.

No measured milliseconds/FPS are reported. Measure with GPU timestamps around
cloud atlas generation and cloud composite, using the same camera, resolution,
cloud settings, release build and warmed-up frames. Compare God Ray off/on with
cloud shadows already on, then cloud shadows off/on separately. Report median
and p95 over at least 120 frames; wall-clock build time is not rendering cost.

Full-resolution fog remains expensive at 4K. Small far-away cloud gaps can
alias with 48 steps; higher-quality future work should move fog to a lower
resolution buffer with depth-aware temporal reconstruction. Existing cloud
history initialization issues are separate from this change and can still
affect overall cloud image quality.
