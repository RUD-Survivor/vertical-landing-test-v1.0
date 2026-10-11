"""Compatibility entry point for the current per-component appearance fit.

The old single-gain annular fit cannot represent the current camera profile.
This proposes coefficients against the latest archived GPU stage.
"""
import subprocess
import sys
from pathlib import Path

root=Path(__file__).resolve().parents[1]
stages=[int(p.name[5:]) for p in (root/'build/tests/solar-chain').glob('stage*') if p.name[5:].isdigit()]
stage=int(sys.argv[1]) if len(sys.argv)>1 else max(stages)
raise SystemExit(subprocess.call([sys.executable,str(root/'tools/solar_chain_fit.py'),str(stage)]))
