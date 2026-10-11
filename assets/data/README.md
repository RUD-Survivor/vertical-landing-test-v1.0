# Sky catalog

`sky-stars.bin` is a transformed subset of HYG v4.1 by David Nash / Astronomy Nexus:
https://github.com/astronexus/HYG-Database/blob/main/hyg/CURRENT/hygdata_v41.csv

Catalog data and this derived binary are licensed **CC BY-SA 4.0**:
https://creativecommons.org/licenses/by-sa/4.0/
Upstream license: https://github.com/astronexus/HYG-Database/blob/main/LICENSE

Changes: exclude the Sun; select visual magnitude <= 6.5; sort by magnitude;
convert J2000 RA/Dec into unit directions (X=RA 0, Y=north, Z=RA 6h);
convert magnitude to relative flux `10^(-0.4*m)`; approximate chromaticity from
B-V and normalize luminance. Missing B-V uses 0.65. No proper motion is applied.
This is not a spectral calibration. The game's magnitude-zero flux scale is
an explicit relative artistic calibration, not absolute photometry.

Source SHA256: `d9f69fd86bbf90a4e4d52b4c5c53eacfa6dfc0bfdef85bfd94f095e0bebe4ebd`.
8,920 records. Rebuild: `python tools/build_sky_catalog.py SOURCE.csv`.
Binary: little-endian uint32 magic `0x53544152`, uint32 count, then 32-byte
records `{float direction[3], relativeFlux, color[3], reserved}`.

## Milky Way background

`milky-way.bin` derives from NASA SVS **Deep Star Maps 2020**, item 4851:
https://svs.gsfc.nasa.gov/4851/
Source: `milkyway_2020_4k.exr`, celestial ICRF/J2000 coordinates, linear half-float.
Credit: **NASA/Goddard Space Flight Center Scientific Visualization Studio.
Gaia DR2: ESA/Gaia/DPAC.** Visualization by Ernie Wright (USRA).
NASA media usage guidelines: https://www.nasa.gov/nasa-brand-center/images-and-media/
No NASA endorsement is implied. This map is separate from the CC BY-SA HYG asset above.

This catalog visualization omits the bright Hipparcos/Tycho foreground. It is not
a photographic panorama or an absolute spectrally calibrated radiance map.
NASA estimates Gaia color balance and renders missing color measurements white.
Modifications: spherical-area-weighted downsample to 2048x1024, fixed linear
radiance multiplier 0.002, 12 spherical-area-weighted mips, RGBA16F packing.
No gamma, local contrast enhancement, fabricated dust or procedural core boost.

Source SHA256: `2eb802d6e68d170b410f766c7fec07f7518619f6b6708fdc81e9302d93e74fdb`.
Derived SHA256: `ca116d397d987c61f1829bc1e24d04a9e9df51dd2d82199ac81b4fd2ec6b2c98`.
Rebuild: `python tools/build_milky_way.py SOURCE.exr` (numpy, OpenEXR required).
Runtime needs only the binary: little-endian uint32 magic `0x594B574D`, width,
height, mip count; tightly packed RGBA half-floats for each level down to 1x1.
22,369,640 bytes including the header. RA zero at map center, increases left;
north at top. World axes match `sky-stars.bin`.
