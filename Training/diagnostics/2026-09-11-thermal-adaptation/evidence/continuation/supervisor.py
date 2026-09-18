"""Bounded continuation from current 550k weights; preserve parent and outputs."""
import os,json,sys,time,subprocess,hashlib,traceback
from pathlib import Path
HERE=Path(__file__).resolve().parent
m=json.loads((HERE/'manifest.json').read_text())
assert m['checkpoint_step']==550000 and Path(m['checkpoint']).name=='parent-550k.pt'
for k in list(os.environ):
    if k.startswith('STINTEGY_AUDIT_') or k=='STINTEGY_DIAG_PIN90':os.environ.pop(k)
os.environ['STINTEGY_HOST_BIN']=m['host'];os.environ['PYTHONDONTWRITEBYTECODE']='1'
os.environ['OMP_NUM_THREADS']='1';os.environ['MKL_NUM_THREADS']='1'
sys.path.insert(0,str(HERE/'python'))
import torch,numpy as np,train
from sac import SacAgent,SacConfig
torch.set_num_threads(1)
def digest(p):return hashlib.sha256(Path(p).read_bytes()).hexdigest()
def status(**data):
    data.update(updated_unix=time.time(),supervisor_pid=os.getpid())
    t=HERE/'status.tmp';t.write_text(json.dumps(data,indent=2));t.replace(HERE/'status.json')
    print(json.dumps(data),flush=True)
def evaluate(path):
    agent=SacAgent(480,2,SacConfig(device='cpu',fixed_alpha=m['fixed_alpha']))
    restored=agent.load(str(path));print('Loaded',path,restored,flush=True)
    r=train.evaluate(agent,batch=12,seed_base=900001,solo=True,track='silverstone',seconds=600,modes=(3,3))
    r['mean_clean_lap']=float(np.mean(r['clean_lap_times'])) if r['clean_lap_times'] else None
    return r
try:
    assert digest(m['checkpoint'])==m['checkpoint_sha256']
    assert digest(m['original_checkpoint'])==m['checkpoint_sha256']
    status(phase='baseline',checkpoint=m['checkpoint'])
    baseline=evaluate(m['checkpoint']);(HERE/'baseline.json').write_text(json.dumps(baseline,indent=2))
    command=[sys.executable,'-u',str(HERE/'train_arm.py'),'--solo','--track','silverstone',
             '--steps',str(m['steps']),'--batch','64','--seed','1','--resume',m['checkpoint'],
             '--fixed-alpha',str(m['fixed_alpha']),'--device','cpu','--updates-per-step','2',
             '--episode-seconds','600','--fixed-episode-start','--eval-every','12500',
             '--eval-seconds','600','--eval-batch','12','--stop-after-stale','999',
             '--log-every','500','--checkpoint-dir',str(HERE/'checkpoints')]
    (HERE/'command.json').write_text(json.dumps(command,indent=2))
    with (HERE/'training.log').open('w') as log:
        child=subprocess.Popen(command,stdout=log,stderr=subprocess.STDOUT,cwd=m['worktree'])
        status(phase='training',child_pid=child.pid,start_step=550000,target_step=575000,checkpoint=m['checkpoint'])
        code=child.wait()
    if code:raise RuntimeError(f'training process exited {code}')
    status(phase='final-evaluation')
    final=HERE/'checkpoints/eval-575000.pt';assert final.exists()
    result=evaluate(final)
    comparison={'baseline':baseline,'final':result,'parent':m['checkpoint'],'final_checkpoint':str(final)}
    (HERE/'comparison.json').write_text(json.dumps(comparison,indent=2))
    assert digest(m['original_checkpoint'])==m['checkpoint_sha256']
    status(phase='complete',result=str(HERE/'comparison.json'))
except BaseException as error:
    status(phase='failed',error=repr(error));traceback.print_exc();raise
