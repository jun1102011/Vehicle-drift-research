"""从实际调度日志核对K/偏置，并调用冻结验收器复算起漂、维持、恢复；只写Step35n审计。"""
import sys,importlib.util,json
from pathlib import Path
import numpy as np
sys.dont_write_bytecode=True
W=Path(__file__).resolve().parents[1];M=W.parent/'step35m_synchronous_exit_20261009';sys.path.insert(0,str(M/'internal'))
from schedule_math import series
def mod(p,n):s=importlib.util.spec_from_file_location(n,p);m=importlib.util.module_from_spec(s);s.loader.exec_module(m);return m
a=mod(M/'internal/analyze_cases.py','reference_reader');metrics=mod(M/'internal/state_only_metrics_v2.py','frozen_metrics')
ds=json.loads((W/'results/details.json').read_text(encoding='utf-8'));checks=[]
for d in ds:
 run=Path(d['source']);used=a.read(run/'used_config.json');e=a.raw.load(run,'repair_entry_diag');diag=a.raw.load(run,'exit_schedule_diag');p=a.raw.load(run,'repair_plant_raw')
 c=a.raw.load(run,'repair_cycle_diag');start=float(c[c[:,1]==4][0,0]);ix=np.flatnonzero((e[:,0]>=start-1e-10)&(e[:,0]<start+15-1e-10));n=series(used['profile'],4,e[ix,0]-start)
 erK=float(np.max(abs(n['K'].transpose(0,2,1).reshape(-1,15)-diag[ix,4:19])));erT=float(np.max(abs(n['offset']-diag[ix,19:23])));assert max(erK,erT)<1e-9
 assert np.array_equal(diag[ix,2],diag[ix,3]);at=np.rint(e[ix,0]/.0005).astype(int);erFx=float(np.max(abs(.32*p[at,9:13]+n['offset']-e[ix,52:56])));assert erFx<1e-7
 met=metrics.evaluate_run(run,a.raw.load);assert met['passed']
 checks.append({'case':d['row']['configuration'],'recorded_K_error':erK,'recorded_offset_error':erT,'load_Fx_offset_error':erFx,'same_reference_K_clock':True,'frozen_protocol_recomputed_PASS':True,'metrics':met})
 print('RECORDED_K_AND_PROTOCOL_PASS',d['row']['configuration'],flush=True)
a.dump(W/'audit/recorded_schedule_protocol.json',{'passed':True,'cases':checks,'vehicle_runs':0})
