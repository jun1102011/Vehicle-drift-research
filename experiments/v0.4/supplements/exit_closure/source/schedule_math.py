"""独立Python调度公式，只用于验证/离线分析，不向车辆发送控制。"""
import json
from pathlib import Path
import numpy as np
W=Path(__file__).resolve().parents[1]
def read(p):return json.loads(Path(p).read_text(encoding='utf-8-sig'))
def smooth(q):q=np.clip(q,0,1);return 10*q**3-15*q**4+6*q**5
def mix(a,b,f):return {k:(1-f)*np.asarray(a[k])+f*np.asarray(b[k]) for k in ['x','v','K','offset']}
def legacy(p,t):
 n=p['exit_nodes'];grid=read(W/'model/repair_support/repair_cycle_bank.json')['grid'];s=p['state_protocol']
 if t<2:return mix(n[0],n[1],smooth(t/2))
 if t<3:return mix(n[1],n[2],smooth(t-2))
 q=min((t-3)/(25/6),1);u=.5*smooth(2*q) if q<.5 else .5+.5*smooth(2*q-1)
 j=min(max(np.searchsorted(grid,u,side='right')-1,0),len(grid)-2);f=(u-grid[j])/(grid[j+1]-grid[j]);return mix(n[j+2],n[j+3],f)
def schedule(p,mode,t):
 if mode==4:
  grid=np.asarray(p['state_protocol']['exit_grid']);u=float(smooth(t/p['state_protocol']['exit_reference_s']))
  j=int(np.clip(np.searchsorted(grid,u,side='right')-1,0,5));f=(u-grid[j])/(grid[j+1]-grid[j])
  return mix(p['exit_nodes'][j],p['exit_nodes'][j+1],f)
 if mode==0:return legacy(p,t)
 if mode==1:return legacy(p,t*43/15)
 dur=43/6 if mode==2 else p['state_protocol']['exit_reference_s'];out=mix(p['exit_nodes'][0],p['exit_nodes'][-1],smooth(t/dur))
 if mode==3:out['K']=legacy(p,t)['K']
 return out
def series(p,mode,t):
 nodes=[schedule(p,mode,float(tt)) for tt in t];return {k:np.stack([n[k] for n in nodes]) for k in ['x','v','K','offset']}
