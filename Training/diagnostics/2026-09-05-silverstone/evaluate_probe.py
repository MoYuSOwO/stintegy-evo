import argparse, json, os, sys, time
from pathlib import Path
import numpy as np
import torch

ROOT = Path(os.environ.get('STINTEGY_DIAG_ROOT', str(Path(__file__).resolve().parent)))
sys.path.insert(0, str(ROOT / 'Training/python'))
from host_env import HostEnv, COMPONENT_NAMES
from sac import Actor

def main():
    p = argparse.ArgumentParser()
    p.add_argument('--names', nargs='+', default=['bestsp1','latestsp1'])
    p.add_argument('--batch', type=int, default=2)
    p.add_argument('--steps', type=int, default=4000)
    p.add_argument('--seed', type=int, default=900000)
    p.add_argument('--out', default='baseline')
    p.add_argument('--stochastic', action='store_true')
    p.add_argument('--steer-filter', type=float, default=0)
    args = p.parse_args()
    torch.set_num_threads(1)
    output = ROOT / 'results' / args.out
    output.mkdir(parents=True, exist_ok=True)
    for name in args.names:
        state = torch.load(ROOT/'checkpoints'/f'{name}.pt',map_location='cpu',weights_only=True)
        actor = Actor(452,2,(512,512,256)).eval()
        actor.load_state_dict(state['actor'])
        torch.manual_seed(12345)
        started=time.monotonic()
        traces=[]; probes=[]; laps=[]; terminals=np.zeros(args.batch,dtype=int)
        last_action=np.zeros((args.batch,2),dtype=np.float32)
        crossed=[None]*args.batch; dirty=np.zeros(args.batch,dtype=bool)
        previous=None
        with HostEnv(batch=args.batch, seed_base=args.seed, solo=True, track='silverstone',episode_seconds=args.steps*0.1+60,quiet=False) as env:
            obs=env.reset()
            initial=obs.copy()
            for step in range(args.steps):
                with torch.no_grad():
                    tensor=torch.from_numpy(obs)
                    a, logp=actor(tensor, deterministic=not args.stochastic)
                    mean, ls=actor.net(tensor).chunk(2,dim=-1)
                action=a.numpy()
                action[:,0]=(1-args.steer_filter)*action[:,0]+args.steer_filter*last_action[:,0]
                last_action=action.copy()
                if step%10==0: probes.append(obs.copy())
                new_obs,reward,done,reason,components,race,final_obs=env.step(action)
                now=(step+1)*0.1
                # Per-lane timer; dirty laps are flagged separately from raw times.
                dirty |= (components[4] < 0) | (components[5] < 0)
                for lane in range(args.batch):
                    if done[lane]:
                        terminals[lane]+=1; crossed[lane]=None; dirty[lane]=False
                    elif previous is not None and np.isfinite(previous[lane]) and race[lane]>previous[lane]:
                        b=int(np.floor(previous[lane]/5891.0)); e=int(np.floor(race[lane]/5891.0))
                        for line in range(b+1,e+1):
                            at=now-0.1+(line*5891.0-previous[lane])/(race[lane]-previous[lane])*0.1
                            if crossed[lane] is not None:
                                laps.append(dict(lane=lane,seconds=at-crossed[lane],finish=at,clean=not bool(dirty[lane])))
                            crossed[lane]=at; dirty[lane]=False
                # Columns: race, speed(pre-step obs), curvature, acceleration, SOC,
                # mean wear, ceiling, allowance, reward components(11), log_std(2), side angle.
                trace=np.column_stack([race,obs[:,232]*100,action,obs[:,214],obs[:,[200,204,208,212]].mean(1),obs[:,226],obs[:,227],components.T,ls.numpy(),obs[:,236],obs[:,[198,202,206,210]].mean(1)*150,obs[:,[199,203,207,211]].mean(1)*150])
                traces.append(trace)
                previous=race.copy(); previous[done]=np.nan
                obs=new_obs
                if (step+1)%1000==0:
                    print(f'{args.out}/{name}: {step+1}/{args.steps}, {time.monotonic()-started:.1f}s',flush=True)
        trace=np.stack(traces)
        np.savez_compressed(output/f'{name}.npz',trace=trace,initial=initial,probes=np.stack(probes))
        summary=dict(name=name,seed=args.seed,batch=args.batch,steps=args.steps,alpha=float(state['log_alpha'].exp().item()),initial_allowance=initial[:,227].tolist(),initial_ceiling=initial[:,226].tolist(),laps=laps,terminals=terminals.tolist(),components_mean={k:float(trace[:,:,8+i].mean()) for i,k in enumerate(COMPONENT_NAMES)},wall_seconds=float(time.monotonic()-started))
        (output/f'{name}.json').write_text(json.dumps(summary,indent=2))
        print(json.dumps({**summary,'laps':{'count':len(laps),'clean':sum(x['clean'] for x in laps),'mean':float(np.mean([x['seconds'] for x in laps])) if laps else None,'best':min([x['seconds'] for x in laps],default=None)}}),flush=True)

if __name__=='__main__': main()
