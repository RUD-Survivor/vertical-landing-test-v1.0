"""Offline NASA SVS 4851 celestial EXR -> linear RGBA16F spherical mip chain.

Requires numpy and OpenEXR; game builds do not download or decode EXR.
"""
import argparse
import hashlib
import struct
from pathlib import Path
import numpy as np

SOURCE_SHA256 = '2eb802d6e68d170b410f766c7fec07f7518619f6b6708fdc81e9302d93e74fdb'
RADIANCE_SCALE = .002  # One fixed relative calibration; no gamma or local contrast.


def row_weights(height):
    edges = np.linspace(-np.pi/2, np.pi/2, height+1)
    return np.diff(np.sin(edges))


def downsample(rgb):
    h, w, _ = rgb.shape
    weights = row_weights(h)
    horizontal = (rgb[:, 0::2] + rgb[:, 1::2]) * .5
    if h == 1:
        return horizontal
    return ((horizontal[0::2]*weights[0::2, None, None] +
             horizontal[1::2]*weights[1::2, None, None]) /
            (weights[0::2]+weights[1::2])[:, None, None])


def build(source, output):
    import OpenEXR
    if hashlib.sha256(source.read_bytes()).hexdigest() != SOURCE_SHA256:
        raise ValueError('Unexpected NASA source revision')
    rgb = OpenEXR.File(str(source)).channels()['RGB'].pixels.astype(np.float64)
    if rgb.shape != (2048, 4096, 3) or not np.isfinite(rgb).all() or (rgb < 0).any():
        raise ValueError('Invalid celestial HDR map')
    rgb = downsample(rgb) * RADIANCE_SCALE  # 2048x1024 runtime map.
    levels = []
    while True:
        rgba = np.concatenate((rgb, np.ones((*rgb.shape[:2], 1))), axis=2)
        levels.append(rgba.astype('<f2').tobytes())
        if rgb.shape[1] == 1:
            break
        rgb = downsample(rgb)
    data = struct.pack('<4I', 0x594B574D, 2048, 1024, len(levels)) + b''.join(levels)
    output.write_bytes(data)
    print(f'{len(levels)} levels; {len(data)} bytes; SHA256 {hashlib.sha256(data).hexdigest()}')


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('source', type=Path)
    parser.add_argument('--output', type=Path, default=Path('assets/data/milky-way.bin'))
    args = parser.parse_args()
    build(args.source, args.output)
