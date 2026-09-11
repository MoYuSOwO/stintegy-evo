"""Train with the pinned source snapshot; use one thread for evaluations."""
import os,json,sys
from pathlib import Path
HERE=Path(__file__).resolve().parent
m=json.loads((HERE/'manifest.json').read_text());threads=m['torch_threads']
os.environ['OMP_NUM_THREADS']=str(threads);os.environ['MKL_NUM_THREADS']=str(threads)
os.environ['PYTHONDONTWRITEBYTECODE']='1'
import torch
torch.set_num_threads(threads)
sys.path.insert(0,str(HERE/'python'))
import train
original_report=train.report
def report(*args,**kwargs):
    before=torch.get_num_threads();torch.set_num_threads(m['evaluation_torch_threads'])
    try:return original_report(*args,**kwargs)
    finally:torch.set_num_threads(before)
train.report=report
raise SystemExit(train.main())
