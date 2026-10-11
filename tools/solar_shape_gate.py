"""First shape gate: compare width diversity and scale to NASA, not spike count.

This gate checks improvement over the archived regular-pupil baseline. It does
not establish absolute photographic fidelity or infer a real lens prescription.
"""
import argparse
import json
from pathlib import Path
import numpy as np

ROOT=Path(__file__).resolve().parents[1]/'build/tests/solar-chain'

def main(stage):
    baseline=json.loads((ROOT/'stage9/shape-comparison.json').read_text())
    current=json.loads((ROOT/f'stage{stage}/shape-comparison.json').read_text())
    def errors(report):
        ref=report['nasa']['width_distributions'];out=report['render']['width_distributions']
        diversity=np.sqrt(np.mean([(out[r]['cv']-ref[r]['cv'])**2 for r in ['2.5','4','6']]))
        scale=np.sqrt(np.mean([(out[r]['median']/ref[r]['median']-1)**2 for r in ['2.5','4','6']]))
        return float(diversity),float(scale)
    before=errors(baseline);after=errors(current)
    assert np.isfinite(after).all(),'invalid shape measurements'
    assert after[0]<before[0]*.7,'width diversity did not approach the reference'
    assert after[1]<before[1],'median width scale regressed'
    # Ensure all requested threshold traces exist; censored lengths remain
    # explicit measurements, not fabricated endpoints or an acceptance target.
    for ray in current['render']['features']:
        assert set(ray['length_degrees'])=={'0.1','0.3','0.5'}
        assert set(ray['length_censored'])=={'0.1','0.3','0.5'}
    result={'baseline_width_cv_rms_error':before[0],'current_width_cv_rms_error':after[0],
            'baseline_relative_median_width_rms_error':before[1],'current_relative_median_width_rms_error':after[1],
            'passed':True,'limits':'First engineering shape improvement gate plus visual inspection; not user acceptance or NASA photographic equivalence. Censored trace lengths do not constrain endpoint fitting.'}
    (ROOT/f'stage{stage}/shape-gate.json').write_text(json.dumps(result,indent=2),encoding='utf-8')
    print('PASS shape improvement:',result)

if __name__=='__main__':
    parser=argparse.ArgumentParser();parser.add_argument('stage',type=int);main(parser.parse_args().stage)
