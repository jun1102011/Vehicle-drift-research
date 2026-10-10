"""五组原生数据离线比较；固定窗口，不补点、不以曲线分离为通过条件。"""
from review_case import *
import csv
CASES=['kf384_kr384','kf192_kr384','kf576_kr384','kf384_kr192','kf384_kr576']
I=mod(B/'step35i_direct_exit_20261007/internal/analyze_exit_comparison.py','domain37b')
def mix(a,b,f):return {k:(1-f)*np.array(a[k]).reshape((3,5) if k=='K' else (-1,))+f*np.array(b[k]).reshape((3,5) if k=='K' else (-1,)) for k in ['x','v','K','offset']}
def sample(nodes,grid,s):
 j=max(0,min(np.searchsorted(grid,s,side='right')-1,len(grid)-2));return mix(nodes[j],nodes[j+1],(s-grid[j])/(grid[j+1]-grid[j]))
def smooth(q):q=np.clip(q,0,1);return 10*q**3-15*q**4+6*q**5
def main():
 out=W/'results/analysis';cache=W/'cache';cache.mkdir(exist_ok=True);rows=[];details=[];windows={k:[] for k in ['entry','reversal','exit']};first_control=None;audits=[]
 for case in CASES:
  m=read(W/f'audit/review_{case}.json');run=W/'results/runs'/f'LeftRight_R3_{case}';use=read(run/'used_config.json');pr=use['profile'];st=pr['state_protocol']
  if first_control is None:first_control=use['controller'];first_profile=pr
  assert use['controller']==first_control and pr==first_profile,'不同刚度初始化了不同控制器'
  p=reader.load(run,'repair_plant_raw');e=reader.load(run,'repair_entry_diag');c=reader.load(run,'repair_cycle_diag');sd=reader.load(run,'exit_schedule_diag');u=reader.load(run,'repair_actuator_command');sus=reader.load(run,'asb_suspension')
  t=p[:,0];x=np.c_[np.hypot(p[:,2],p[:,3])/3.6,p[:,4],np.deg2rad(p[:,5])];ci=np.clip(np.rint(t/.0005).astype(int)//20,0,len(e)-1);ref=e[ci,16:19].copy();ref[:,1]*=180/np.pi
  starts={int(ph):float(c[c[:,1]==ph][0,0]) for ph in np.unique(c[:,1])};err=[]
  for i in range(len(c)):
   ph=int(c[i,1]);tau=c[i,0]-starts[ph]
   if ph==7:expected=sample(st['reversal_nodes'],st['reversal_grid'],smooth(tau/st['reversal_s']))
   elif ph==10:expected=sample(st['right_exit_nodes'],st['exit_grid'],smooth(tau/st['exit_reference_s']))
   else:continue
   err.extend([abs(e[i,16:23]-expected['x']).max(),abs(e[i,23:26]-expected['v']).max(),abs(sd[i,4:19]-expected['K'].reshape(-1,order='F')).max(),abs(sd[i,19:23]-expected['offset']).max()])
  assert max(err,default=0)<1e-8
  audits.append(dict(case=case,schedule_max_abs_error=max(err,default=0),controller_equal=True,profile_equal=True,phase_starts=starts))
  domain=I.domain.audit(run,reader.load);dump(out/f'{case}_suspension_domain.json',domain)
  cfg=use['controller']['entry'];lo=np.c_[e[:,23]-cfg['steer_deviation'],np.full(len(e),cfg['slip_low'][0]),np.full(len(e),cfg['slip_low'][1])];hi=np.c_[e[:,23]+cfg['steer_deviation'],np.full(len(e),cfg['slip_high'][0]),np.full(len(e),np.inf)]
  clipped=np.clip(e[:,26:29],lo,hi);amp=abs(e[:,26:29]-clipped)>1e-9;slew=abs(e[:,29:32]-clipped)>1e-9
  dd=dict(case=case,windows={},actual_end_s=float(t[-1]),terminal_state_V_beta_deg_AVz=x[-1],right_hold_observed_s=max(0,float(t[-1]-starts[9])) if 9 in starts and 10 not in starts else None)
  for name,ph,duration in [('entry',1,8),('reversal',7,15),('exit',10,15)]:
   start=starts.get(ph);n=int(round(duration/.0005));ix=int(round(start/.0005)) if start is not None else len(t);end=ix+n
   if end>len(t):dd['windows'][name]=dict(complete=False,start_s=start);continue
   sl=slice(ix,end);ct=(e[:,0]>=start-1e-10)&(e[:,0]<start+duration-1e-10);fxz=p[sl,17:21];loc=np.unravel_index(fxz.argmin(),fxz.shape);error=x[sl]-ref[sl]
   post=cfg['steering_rate_rad_s']*180/np.pi;steer_rate=abs(u[sl,3:5]);on=steer_rate>=post-1e-6
   z=dict(complete=True,start_s=start,duration_s=duration,samples=n,own_J=np.sum(error**2,axis=0)*.0005,own_rmse=np.sqrt(np.mean(error**2,axis=0)),own_peak=abs(error).max(axis=0),V_min=float(x[sl,0].min()),V_max=float(x[sl,0].max()),beta_min=float(x[sl,1].min()),beta_max=float(x[sl,1].max()),AVz_min=float(x[sl,2].min()),AVz_max=float(x[sl,2].max()),min_Fz_N=float(fxz.min()),min_Fz_wheel=['FL','FR','RL','RR'][loc[1]],min_Fz_tau_s=float(loc[0]*.0005),torque_rms=float(np.sqrt(np.mean(u[sl,5:9]**2))),torque_peak=float(abs(u[sl,5:9]).max()),torque_margin_Nm=float(3000-abs(u[sl,5:9]).max()),steer_rate_peak_deg_s=float(steer_rate.max()),steer_rate_limit_s=float(np.any(on,axis=1).sum()*.0005),steer_rate_limit_longest_s=longest(np.any(on,axis=1))*.0005,amplitude_clip_s=amp[ct].sum(axis=0)*.01,slew_clip_s=slew[ct].sum(axis=0)*.01,request_budget_max=abs(e[ct,26:29]-e[ct,29:32]).max(axis=0),wheel_projection_s=float((e[ct,65]>.5).sum()*.01),torque_protection_s=float((e[ct,66]>.5).sum()*.01))
   if name=='exit':z['final_target_J']=np.sum((x[sl]-[12,0,0])**2,axis=0)*.0005
   pre=max(0,ix-2000);z['pre_1s_mean']=x[pre:ix].mean(axis=0);z['pre_1s_peak_to_peak']=np.ptp(x[pre:ix],axis=0)
   dd['windows'][name]=z;windows[name].append(dict(case=case,x=x[sl].copy(),ref=ref[sl].copy()))
   arr=np.c_[np.arange(n)*.0005,x[sl],ref[sl],u[sl,1:9],p[sl,27:31],p[sl,9:21],sus[sl,25:27],sus[sl,1:3]]
   np.savetxt(cache/f'{case}_{name}.csv',arr[::20],delimiter=',',fmt='%.17g')
   ctl=np.c_[e[ct,0]-start,e[ct,23:32],amp[ct],slew[ct],e[ct,64:67]];np.savetxt(cache/f'{case}_{name}_control.csv',ctl,delimiter=',',fmt='%.17g')
  dd['events']={}
  for name,ph,target,tol,count,ev in [('left',1,st['target'],[.2,2,.03],600,m['first_drift']['entry']),('right',7,st['right_target'],[.2,2,.03],600,m['second_drift']['entry']),('recovery',10,[12,0,0],[.2,1,.02],2000,m['recovery'])]:
   target=np.array(target,float);target[1]*=180/np.pi;start=starts.get(ph);state_ok=abs(x-target)<=tol;single=[]
   for j in range(3):
    se=event(t,state_ok[:,j]&(t>=(start if start is not None else np.inf)),count)
    if se:se['elapsed_s']=se['start_s']-start
    single.append(se)
   last=[]
   if ev and ev['index']>0:last=[['V','beta','AVz'][j] for j in range(3) if not state_ok[ev['index']-1,j]]
   dd['events'][name]=dict(single_state_events=single,last_limiting_states=last,joint_event=ev)
  dd['recovery_after_confirmation_outside_samples']=(m['recovery'] or {}).get('reexit_samples_after_confirm');details.append(dd)
  h1=m['first_drift']['hold'] or {};h2=m['second_drift']['hold'] or {};a=m['first_drift']['entry'] or {};b=m['second_drift']['entry'] or {};r=m['recovery'] or {};turn=dd['windows']['reversal'];ex=dd['windows']['exit']
  row=dict(case=case,Kf=use['vehicle_config']['kf_deg'],Kr=use['vehicle_config']['kr_deg'],classification=m['classification'],left_entry_s=a.get('elapsed_s'),right_entry_from_reversal_s=b.get('elapsed_s'),recovery_s=r.get('elapsed_s'),left_beta_rmse_deg=(h1.get('rmse') or [None]*3)[1],right_beta_rmse_deg=(h2.get('rmse') or [None]*3)[1],left_radius_m=h1.get('natural_radius_m'),right_radius_m=h2.get('natural_radius_m'),reversal_Vmin=turn.get('V_min'),reversal_AVz_min=turn.get('AVz_min'),reversal_minFz=turn.get('min_Fz_N'),reversal_torque_RMS=turn.get('torque_rms'),reversal_torque_peak=turn.get('torque_peak'),exit_Vmin=ex.get('V_min'),exit_minFz=ex.get('min_Fz_N'),exit_torque_RMS=ex.get('torque_rms'),full_minFz=m['min_Fz_N'],failure_reasons=';'.join(m['failure_reasons']))
  row['actual_end_s']=m['actual_end_s'];rows.append(row);np.savetxt(cache/f'{case}_trajectory.csv',np.c_[t,p[:,35:37],c[ci,1]][::20],delimiter=',',fmt='%.17g')
 separation={}
 for name,items in windows.items():
  if len(items)<2:separation[name]=dict(complete_five=False,available_cases=[z['case'] for z in items]);continue
  xx=np.stack([z['x'] for z in items]);rr=np.stack([z['ref'] for z in items]);span=np.ptp(xx,axis=0);reference_span=np.ptp(rr,axis=0)
  separation[name]=dict(complete_five=len(items)==5,same_time_state_range_peak=span.max(axis=0),same_time_state_range_rms=np.sqrt(np.mean(span**2,axis=0)),reference_range_peak=reference_span.max(axis=0),reference_equal=bool(np.all(reference_span<=1e-10)),reference_equality_tolerance=1e-10,reference_exact_equal=bool(np.all(reference_span==0)),common_baseline_reference_J=np.sum((xx-rr[0])**2,axis=1)*.0005,cases=[z['case'] for z in items])
 dump(out/'details.json',details);dump(out/'separation.json',separation);dump(W/'audit/schedule_and_controller_validation.json',dict(passed=True,cases=audits))
 with (out/'summary.csv').open('w',encoding='utf-8-sig',newline='') as f:wr=csv.DictWriter(f,fieldnames=list(rows[0]));wr.writeheader();wr.writerows(rows)
 dump(out/'summary.json',rows);print(json.dumps(rows,ensure_ascii=False))
if __name__=='__main__':main()
