import json, os, sys
from pathlib import Path
import numpy as np

ROOT=Path(os.environ.get('STINTEGY_DIAG_ROOT',str(Path(__file__).resolve().parent)))
sys.path.insert(0,str(ROOT/'Training/python'))
from nstep import NStepBatcher

def verify_timing():
    before=[json.loads(x) for x in (ROOT/'timing.jsonl').read_text().splitlines()]
    after=[json.loads(x) for x in (ROOT/'timing-aligned.jsonl').read_text().splitlines()]
    assert before[-1]['sameReturnedObservation']
    assert before[-1]['positiveActionPostSpeed'] != before[-1]['negativeActionPostSpeed']
    braking=before[1]
    assert all(x['accel']>0 for x in braking['frames'][:5])
    assert braking['frames'][5]['accel']<0
    assert braking['returnedLastAction']==1 and braking['command']==-1
    assert not after[-1]['sameReturnedObservation']
    for row in after[:-1]:
        assert abs(row['returnedSpeed']-row['after'])<1e-5
        assert row['returnedLastAction']==row['command']
        assert all(x['accel']*row['command']>0 for x in row['frames'])
    return 'confirmed original phase mismatch; isolated aligned branch satisfies action/post-state timing'

def verify_nstep():
    b=NStepBatcher(2,3,.9); rows=[]
    for i in range(3):
        out=b.add(np.array([[i],[100+i]],np.float32),np.array([[i],[i]],np.float32),
            np.array([i+1,10*(i+1)],np.float32),np.array([[i+1],[101+i]],np.float32),
            np.array([i==1,i==2]),np.array([False,i==2]))
        if out is not None: rows.extend(list(zip(*out)))
    lookup={int(row[0][0]):row for row in rows}
    assert np.isclose(lookup[0][2],1+.9*2) and lookup[0][4]==0 and lookup[0][5]==2 and lookup[0][3][0]==2
    assert np.isclose(lookup[100][2],10+.9*20+.9**2*30) and lookup[100][4]==1 and lookup[100][5]==3 and lookup[100][3][0]==103
    assert lookup[1][4]==0 and lookup[101][4]==1
    return 'discounted reward, timeout bootstrap, terminal, lane separation, tail horizon passed'

result={'timing':verify_timing(),'nstep':verify_nstep()}
(ROOT/'verification.json').write_text(json.dumps(result,indent=2))
print(json.dumps(result,indent=2))
