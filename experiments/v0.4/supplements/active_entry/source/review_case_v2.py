"""独立复核时序、实际力偶、原生验收；技术错误不自动放行。"""
import sys,json,hashlib,importlib.util
from pathlib import Path
import numpy as np
sys.dont_write_bytecode=True
W=Path(__file__).resolve().parents[1];B=W.parent
def read(p):return json.loads(p.read_text(encoding='utf-8-sig'))
def sha(p):return hashlib.sha256(p.read_bytes()).hexdigest()
def dump(p,x):p.parent.mkdir(parents=True,exist_ok=True);p.write_text(json.dumps(x,ensure_ascii=False,indent=2,default=lambda a:a.item() if isinstance(a,np.generic)else a.tolist()),encoding='utf-8')
def mod(p,n):s=importlib.util.spec_from_file_location(n,p);m=importlib.util.module_from_spec(s);s.loader.exec_module(m);return m
raw=mod(B/'step33c_asb_amplitude_20261005/internal/data_archive.py','native_reader')
metrics=mod(W/'internal/state_only_metrics_v2.py','acceptance')
def audit(case):
 pid='V12_beta30_state_only_R3_S';run=W/'results/runs'/(pid+'_'+case)
 old=B/'step35m_synchronous_exit_20261009/results/runs/V12_beta30_state_only_R3_S_kf384_kr384' if case=='A' else W/'results/runs'/(pid+'_A')
 idx=read(run/'archive_index.json');oidx=read(old/'archive_index.json')
 assert idx['channel_count']==57 and idx['bitwise_readback'] and sha(run/'raw_numeric.mat')==idx['archive_sha256']
 assert sha(old/'raw_numeric.mat')==oidx['archive_sha256']
 e=raw.load(run,'repair_entry_diag');p=raw.load(run,'repair_plant_raw');c=raw.load(run,'repair_cycle_diag');pd=raw.load(run,'asb_entry_pulse_diag');im=raw.load(run,'asb_import12');ad=raw.load(run,'asb_diag');s=raw.load(run,'asb_suspension');ac=raw.load(run,'repair_actuator_command')
 start=float(e[e[:,1]==1][0,0]);cut=start+1.5;checks=[]
 for name in oidx['channels']:
  if name=='asb_entry_pulse_diag':continue
  a=raw.load(run,name);b=raw.load(old,name)
  if case!='A':a=a[a[:,0]<cut-1e-10];b=b[b[:,0]<cut-1e-10]
  same=a.shape==b.shape and np.array_equal(a.view(np.uint64),b.view(np.uint64))
  signed=0
  if not same:
   assert case!='A' and a.shape==b.shape and name in ['asb_diag','asb_import12'],('regression',case,name)
   bad=a.view(np.uint64)!=b.view(np.uint64)
   allowed=[7,15,16,19,20,29,33,35,8,17,18,21,22,30,34,36] if name=='asb_diag' else [9,10,11,12]
   assert np.array_equal(a,b) and np.all((a[bad]==0)&(b[bad]==0)) and set(np.where(bad)[1]).issubset(allowed),'Not exclusively signed zero in ASB diagnostics/imports'
   signed=int(bad.sum())
  checks.append({'channel':name,'samples':len(a),'bitwise_equal':same,'numeric_equal':True,'signed_zero_only_bits':signed})
 assert len(checks)==56
 t=p[:,0];n=len(t);assert np.max(abs(np.diff(t)-.0005))<1e-10
 assert np.array_equal(pd[:n,0],t) and np.array_equal(im[:,0],t)
 pd=pd[:n];ad=ad[:n];tau=t-start;g=np.zeros(n);dg=np.zeros(n)
 rise=(tau>1.5)&(tau<1.6);flat=(tau>=1.6)&(tau<=1.9);fall=(tau>1.9)&(tau<2)
 g[rise]=.5*(1-np.cos(np.pi*(tau[rise]-1.5)/.1));dg[rise]=np.pi*np.sin(np.pi*(tau[rise]-1.5)/.1)/.2;g[flat]=1
 g[fall]=.5*(1-np.cos(np.pi*(2-tau[fall])/.1));dg[fall]=-np.pi*np.sin(np.pi*(2-tau[fall])/.1)/.2
 amp=np.asarray(next(x for x in read(W/'source/cases.json')if x['id']==case)['moment_Nm']);expected=g[:,None]*amp
 assert np.max(abs(pd[t>=start,2]-start))<1e-10
 assert np.max(abs(pd[:,7:9]-expected))<1e-8 and np.max(abs(ad[:,7:9]-expected))<1e-8
 assert np.max(abs(pd[:,9:11]-dg[:,None]*amp))<1e-7
 assert np.array_equal(im[:,:9],ac)
 lever=1.675-np.c_[s[:,21]+s[:,22],s[:,23]+s[:,24]]*.001
 q=expected/lever;wheel=np.c_[q[:,0],-q[:,0],q[:,1],-q[:,1]];forces=wheel/[.959,.959,.797,.797]
 err=float(abs(im[:,9:13]-forces[:,[0,2,1,3]]).max());assert err<1e-7,('force mapping',err)
 recv=s[:,5:9];recv_err=float(abs(recv-forces).max());lag_err=float(abs(recv[1:]-forces[:-1]).max())
 assert min(recv_err,lag_err)<1e-7,('unexplained receive timing',recv_err,lag_err)
 # 原生实收悬架力经运动比回算轮端力偶；无需把实际接地Fz等同这个输入。
 rw=recv*[.959,.959,.797,.797];net=np.c_[rw[:,0]+rw[:,1],rw[:,2]+rw[:,3]];assert abs(net).max()<1e-8
 actual=np.c_[rw[:,0]-rw[:,1],rw[:,2]-rw[:,3]]*lever/2
 assert abs(ad[:,9:11]-actual).max()<1e-7
 used=read(run/'used_config.json');assert used['vehicle_config']['kf_deg']==used['vehicle_config']['kr_deg']==384
 assert abs(s[:,25:27]-s[:,1:3]*384).max()<1e-8
 # 原有五状态请求与预算执行按实际记录重建。
 prof=used['profile'];sched=raw.load(run,'exit_schedule_diag');phase=c[:,1];ci=np.minimum(np.rint(t/.0005).astype(int)//20,len(e)-1)
 assert abs(ac[:,5:9]-e[ci,60:64]).max()<1e-8
 cfg=used['controller']['entry'];ff=e[:,23:26];lo=np.c_[ff[:,0]-cfg['steer_deviation'],np.full(len(e),cfg['slip_low'][0]),np.full(len(e),cfg['slip_low'][1])];hi=np.c_[ff[:,0]+cfg['steer_deviation'],np.full(len(e),cfg['slip_high'][0]),np.full(len(e),np.inf)]
 clipped=np.clip(e[:,26:29],lo,hi);rates=np.array([cfg['steering_rate_rad_s'],cfg['slip_rate'],cfg['slip_rate']]);assert abs(e[1:,29:32]-np.clip(clipped[1:],e[:-1,29:32]-.01*rates,e[:-1,29:32]+.01*rates)).max()<1e-9
 met=metrics.evaluate_run(run,raw.load)
 if case=='A':
  om=read(B/'step35m_synchronous_exit_20261009/results/metrics/V12_beta30_state_only_R3_S_kf384_kr384.json')
  for name in ['entry','phase3_hold','phase4']:assert met[name]==om[name],name
 # 验收独立的卷积计数，与原循环计数相比。
 x=np.c_[np.hypot(p[:,2],p[:,3])/3.6,p[:,4],np.deg2rad(p[:,5])]
 mask=np.all(abs(x-[12,-30,.5989797412715093])<=[.2,2,.03],axis=1)&(x[:,2]>0)&(t>=start)
 sums=np.r_[0,np.cumsum(mask.astype(int))];hits=np.flatnonzero(sums[600:]-sums[:-600]==600)
 if met['entry']['first600']:assert abs(t[hits[0]]-met['entry']['first600']['start_s'])<1e-10
 else:assert not len(hits)
 exit_start=met['phase4']['actual_start_s']
 if exit_start is not None:
  mask=np.all(abs(x-[12,0,0])<=[.2,1,.02],axis=1)&(t>=exit_start);ss=np.r_[0,np.cumsum(mask.astype(int))];hits=np.flatnonzero(ss[2000:]-ss[:-2000]==2000)
  ev=met['phase4']['first_straight2000'];assert (bool(len(hits))==bool(ev))
  if ev:assert abs(t[hits[0]]-ev['start_s'])<1e-10
 dump(W/'results/metrics'/f'{case}.json',met)
 out={'passed':True,'case':case,'full_baseline_regression':case=='A','regression_channels':checks,'historical_source':str(old),'performance_passed':met['passed'],'classification':met['classification'],'entry_start_s':start,'raw_sha256':idx['archive_sha256'],'pulse_mapping_max_error_N':err,'received_same_sample_max_error_N':recv_err,'received_one_sample_lag_max_error_N':lag_err,'received_timing':'same sample' if recv_err<1e-7 else 'one native sample delay','axis_net_force_max_N':float(abs(net).max()),'external_ASB_waveform_verified':True,'native_events_independently_verified':True,'review_source_sha256':sha(Path(__file__)),'signed_zero_scope':'ASB diagnostics and external four inputs only; original bit patterns preserved'}
 dump(W/'audit'/f'review_{case}.json',out);print(json.dumps({k:v for k,v in out.items()if k!='regression_channels'},ensure_ascii=False))
 return out
if __name__=='__main__':audit(sys.argv[1])
