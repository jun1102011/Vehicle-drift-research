"""从原生记录独立核验左右状态任务；不运行车辆，不补值。"""
from pathlib import Path
import sys,json,hashlib,importlib.util
import numpy as np
sys.dont_write_bytecode=True
W=Path(__file__).resolve().parents[1];B=W.parent
def read(p):return json.loads(p.read_text(encoding='utf-8-sig'))
def sha(p):return hashlib.sha256(p.read_bytes()).hexdigest()
def dump(p,v):p.parent.mkdir(parents=True,exist_ok=True);p.write_text(json.dumps(v,ensure_ascii=False,indent=2,default=lambda a:a.item() if isinstance(a,np.generic) else a.tolist()),encoding='utf-8')
def mod(p,n):s=importlib.util.spec_from_file_location(n,p);m=importlib.util.module_from_spec(s);s.loader.exec_module(m);return m
reader=mod(B/'step33c_asb_amplitude_20261005/internal/data_archive.py','reader37')
oldmetrics=mod(B/'step35m_synchronous_exit_20261009/internal/state_only_metrics_v2.py','metrics37')
oldmetrics.W=W # 只将历史验收器的profile查找根指向本批；公式、门槛不变。
def event(t,mask,n):
 ss=np.r_[0,np.cumsum(mask.astype(int))];hits=np.flatnonzero(ss[n:]-ss[:-n]==n)
 if not len(hits):return None
 i=int(hits[0]);return dict(start_s=float(t[i]),confirmed_s=float(t[i+n-1]),index=i)
def longest(mask):
 d=np.diff(np.r_[False,mask,False].astype(int));starts=np.flatnonzero(d==1);ends=np.flatnonzero(d==-1);return int(max(ends-starts,default=0))
def review(case):
 pid='LeftRight_R3'
 run=W/'results/runs'/f'{pid}_{case}';idx=read(run/'archive_index.json');assert sha(run/'raw_numeric.mat')==idx['archive_sha256'] and idx['bitwise_readback']
 use=read(run/'used_config.json');pr=use['profile'];state=pr['state_protocol'];combo=state['bidirectional']>0
 p=reader.load(run,'repair_plant_raw');e=reader.load(run,'repair_entry_diag');c=reader.load(run,'repair_cycle_diag');u=reader.load(run,'repair_actuator_command');imp=reader.load(run,'asb_import12');sus=reader.load(run,'asb_suspension');sch=reader.load(run,'exit_schedule_diag')
 t=p[:,0];assert abs(np.diff(t)-.0005).max()<1e-10 and np.array_equal(u[:,0],t) and np.array_equal(imp[:,0],t)
 assert np.array_equal(u,imp[:,:9]) and np.all(imp[:,9:]==0) and abs(sus[:,25:27]-np.array([use['vehicle_config']['kf_deg'],use['vehicle_config']['kr_deg']])*sus[:,1:3]).max()<1e-8
 x=np.c_[np.hypot(p[:,2],p[:,3])/3.6,p[:,4],np.deg2rad(p[:,5])];ci=np.clip(np.rint(t/.0005).astype(int)//20,0,len(e)-1)
 assert abs(u[:,5:9]-e[ci,60:64]).max()<1e-8
 # 用逐控制步日志重建五项反馈、移动幅值边界及原限速公式。
 K=sch[:,4:19].reshape(-1,5,3).transpose(0,2,1);req=e[:,23:26]-np.einsum('nij,nj->ni',K,e[:,9:14]-e[:,16:21]);assert abs(req-e[:,26:29]).max()<1e-8
 cfg=use['controller']['entry'];lo=np.c_[e[:,23]-cfg['steer_deviation'],np.full(len(e),cfg['slip_low'][0]),np.full(len(e),cfg['slip_low'][1])];hi=np.c_[e[:,23]+cfg['steer_deviation'],np.full(len(e),cfg['slip_high'][0]),np.full(len(e),np.inf)]
 clip=np.clip(req,lo,hi);rate=np.array([cfg['steering_rate_rad_s'],cfg['slip_rate'],cfg['slip_rate']]);assert abs(e[1:,29:32]-np.clip(clip[1:],e[:-1,29:32]-.01*rate,e[:-1,29:32]+.01*rate)).max()<1e-8
 summary=read(run/'summary.json');complete=read(run/'completion.json');reasons=[]
 def firstphase(n):
  hits=np.flatnonzero(c[:,1]==n);return float(c[hits[0],0]) if len(hits) else None
 def assess_side(target,ep,hp,nextphase):
  target=np.array(target,float);target[1]=np.rad2deg(target[1]);start=firstphase(ep);hs=firstphase(hp);end=firstphase(nextphase)
  mask=np.all(abs(x-target)<=[.2,2,.03],axis=1)&(x[:,2]*np.sign(target[2])>0)&(t>=(start if start is not None else np.inf))
  if end is not None:mask&=t<end
  ev=event(t,mask,600);hm=None
  if ev:ev['elapsed_s']=ev['start_s']-start
  if hs is not None:
   i=int(round(hs/.0005));j=i+20000
   if j<=len(t) and end is not None and abs(end-hs-10)<1e-8:
    err=x[i:j]-target;bad=np.any(abs(err)>[.2,2,.03],axis=1);prot=e[ci[i:j],66]>.5
    rm=np.sqrt(np.mean(err**2,axis=0));q=p[i:j,35:37];q=q-q.mean(axis=0);fit=np.linalg.lstsq(np.c_[2*q,np.ones(len(q))],np.sum(q*q,axis=1),rcond=None)[0];radius=np.sqrt(max(0,fit[2]+fit[0]**2+fit[1]**2))
    hm=dict(start_s=hs,end_exclusive_s=hs+10,samples=20000,rmse=rm,peak_error=abs(err).max(axis=0),out_of_band_s=float(bad.sum()*.0005),longest_out_of_band_s=longest(bad)*.0005,torque_protection_fraction=float(prot.mean()),protection_longest_s=longest(prot)*.0005,min_Fz=float(p[i:j,17:21].min()),natural_radius_m=float(radius),beta_mean_deg=float(x[i:j,1].mean()),passed=bool(np.all(rm<=[.2,2,.03]) and prot.mean()<=.05 and longest(prot)*.0005<=.2 and p[i:j,17:21].min()>0))
  return dict(start_s=start,entry=ev,hold=hm,passed=bool(ev and hm and hm['passed']))
 first=assess_side(state['target'],1,3,7 if combo else 4);second=assess_side(state['right_target'],7,9,10) if combo else None
 es=firstphase(10 if combo else 4);sm=np.all(abs(x-[12,0,0])<=[.2,1,.02],axis=1);ev=event(t,sm&(t>=(es if es is not None else np.inf)),2000)
 if ev:ev['elapsed_s']=ev['start_s']-es;ev['reexit_samples_after_confirm']=int((~sm[int(round(ev['confirmed_s']/.0005))+1:]).sum())
 if not first['passed']:reasons.append('first_drift_failed')
 if combo and not second['passed']:reasons.append('right_hold_incomplete' if second['entry'] and second['hold'] is None else 'right_reversal_drift_failed')
 if not ev:reasons.append('straight_recovery_not_reached' if es is None else 'straight_recovery_missing')
 if summary['error'] or not complete['vehicle_completed']:
  reasons.append('vehicle_termination')
  log=(run/'LastRun_log.txt.evidence').read_text(encoding='utf-8',errors='replace')
  if 'S and L values for path could not be calculated' in log:reasons.append('carsim_road_coordinate_failure')
 if p[:,17:21].min()<=0:reasons.append('nonpositive_wheel_load')
 if abs(u[:,5:9]).max()>3000+1e-6:reasons.append('torque_budget_violation')
 regression=[]
 # 每组与同刚度旧记录核对共同有效前缀，不裁掉旧末端异常样本。
 old=B/'step37b_bidirectional_stiffness_20261010/results/runs'/f'LeftRight_R3_{case}'
 oi=read(old/'archive_index.json');assert sha(old/'raw_numeric.mat')==oi['archive_sha256']
 differences=[]
 for name in oi['channels']:
  aa=reader.load(run,name);bb=reader.load(old,name)
  n=min(len(aa),len(bb));same=aa.shape[1:]==bb.shape[1:] and np.array_equal(aa[:n].view(np.uint64),bb[:n].view(np.uint64))
  info=dict(channel=name,old_shape=bb.shape,new_shape=aa.shape,common_rows=n,bitwise_equal=same)
  if not same:
   info['max_abs_difference']=float(np.max(np.abs(aa[:n]-bb[:n])))
   info['first_different_row']=int(np.flatnonzero(np.any(aa[:n].view(np.uint64)!=bb[:n].view(np.uint64),axis=1))[0]) if aa.ndim==2 else None
   differences.append(info)
  else:regression.append(name)
 dump(W/'audit'/f'road_regression_{case}.json',dict(case=case,identical_channels=regression,differences=differences,all_common_bitwise_equal=not differences))
 assert not differences,('road_coordinate_changed_vehicle_response',differences)
 if case=='kf384_kr384':assert len(t)==120001 and len(regression)==56
 result=dict(case=case,profile=pid,technical_pass=True,performance_pass=not reasons,classification='PASS' if not reasons else ('PARTIAL' if first['entry'] and not any(z in reasons for z in ['vehicle_termination','nonpositive_wheel_load','torque_budget_violation']) else 'FAIL'),failure_reasons=reasons,first_drift=first,second_drift=second,exit_start_s=es,recovery=ev,full_regression_channels=regression,actual_end_s=float(t[-1]),raw_sha256=idx['archive_sha256'],V_min=float(x[:,0].min()),V_max=float(x[:,0].max()),min_Fz_N=float(p[:,17:21].min()),torque_RMS_Nm=float(np.sqrt(np.mean(u[:,5:9]**2))),torque_peak_Nm=float(abs(u[:,5:9]).max()),external_ASB_zero=True,request_formula_verified=True)
 dump(W/'audit'/f'review_{case}.json',result);dump(W/'results/metrics'/f'{case}.json',result)
 print(json.dumps(result,ensure_ascii=False,default=lambda a:a.tolist()),flush=True);return result
if __name__=='__main__':review(sys.argv[1])
