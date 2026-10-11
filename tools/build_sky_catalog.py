"""Build the attributed HYG v4.1 subset; never downloads during a game build."""
import argparse
import csv
import hashlib
import math
import struct
from pathlib import Path

SOURCE_SHA256 = "d9f69fd86bbf90a4e4d52b4c5c53eacfa6dfc0bfdef85bfd94f095e0bebe4ebd"


def build(source, output):
    if hashlib.sha256(source.read_bytes()).hexdigest() != SOURCE_SHA256:
        raise ValueError("Unexpected HYG source: pin/review a new revision before rebuilding")
    stars = []
    with source.open(encoding="utf-8") as stream:
        for row in csv.DictReader(stream):
            if row["id"] == "0":  # The Sun is rendered by the celestial system.
                continue
            magnitude = float(row["mag"])
            if magnitude > 6.5:
                continue
            ra = float(row["ra"]) * math.pi / 12
            dec = math.radians(float(row["dec"]))
            bv = float(row["ci"]) if row["ci"] else .65
            t = max(0., min(1., (bv + .3) / 2.3))
            # Approximate display chromaticity from B-V, not spectrometry.
            rgb = [a * (1 - t) + b * t for a, b in zip((.65, .78, 1.), (1., .65, .35))]
            luminance = sum(a * b for a, b in zip(rgb, (.2126, .7152, .0722)))
            direction = (math.cos(dec) * math.cos(ra), math.sin(dec), math.cos(dec) * math.sin(ra))
            stars.append((magnitude, (*direction, 10 ** (-.4 * magnitude), *[v / luminance for v in rgb], 1.)))
    stars.sort(key=lambda item: item[0])
    output.parent.mkdir(parents=True, exist_ok=True)
    with output.open("wb") as stream:
        stream.write(struct.pack("<II", 0x53544152, len(stars)))
        for _, record in stars:
            stream.write(struct.pack("<8f", *record))
    print(f"Built {len(stars)} stars: {output}")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("source", type=Path)
    parser.add_argument("--output", type=Path, default=Path("assets/data/sky-stars.bin"))
    args = parser.parse_args()
    build(args.source, args.output)
