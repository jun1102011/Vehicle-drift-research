"""只读五组唯一原始记录，原生指标与小型图缓存；不启动车辆。"""
import sys,csv,json,hashlib
from pathlib import Path
import numpy as np
sys.dont_write_bytecode=True
from review_case import W,B,raw,read,dump,mod,metrics
I=mod(B/'step35i_direct_exit_20261007/internal/analyze_exit_comparison.py','history_analysis')
DOM=I.domain;OUT=W/'results/analysis';CACHE=W/'cache';DT=.0005
IDS=['A','F_plus','F_minus','R_plus','R_minus'];WH=['FL','FR','RL','RR'];TARGET=np.array([12,-30,.5989797412715093])
def span(mask,dt=DT):
 edges=np.diff(np.r_[False,mask,False].astype(int));runs=np.flatnonzero(edges==-1)-np.flatnonzero(edges==1)
 return {'total_s':float(mask.sum()*dt),'longest_s':float(max(runs,default=0)*dt)}
def stat(x,ref,dt=DT):
 er=x-ref;return {'J':(np.sum(er**2,axis=0)*dt).tolist(),'RMSE':np.sqrt(np.mean(er**2,axis=0)).tolist(),'peak':np.max(abs(er),axis=0).tolist()}
def cache(name,x):np.savetxt(CACHE/(name+'.csv'),x,delimiter=',',fmt='%.17g')
def table(p,rows):
 with p.open('w',encoding='utf-8-sig',newline='')as f:w=csv.DictWriter(f,fieldnames=list(rows[0]));w.writeheader();w.writerows(rows)
def planar(run):
 t,v,alias=DOM.vs_read(run);suffix=['L1','R1','L2','R2']
 def get(n,u):
  assert all(alias[n+'_'+s][1]==u for s in suffix)
  return np.c_[[v[:,alias[n+'_'+s][0]].astype(float)for s in suffix]].T
 fx=get('Fx','N');fy=get('Fy','N');delta=np.deg2rad(get('Steer','deg'));mz=get('Mz','N-m')
 bx=fx*np.cos(delta)-fy*np.sin(delta);by=fx*np.sin(delta)+fy*np.cos(delta)
 comp=np.c_[1.177744981*by[:,:2].sum(1)-1.682255019*by[:,2:].sum(1),1.675/2*(-bx[:,0]+bx[:,1]-bx[:,2]+bx[:,3]),mz.sum(1)]
 return t,np.c_[bx[:,:2].sum(1),bx[:,2:].sum(1),by[:,:2].sum(1),by[:,2:].sum(1),comp,comp.sum(1)]
def main():
 OUT.mkdir(parents=True,exist_ok=True);CACHE.mkdir(exist_ok=True);details=[];rows=[];samples=[];base=None;domains=[]
 for cid in IDS:
  run=W/'results/runs'/('V12_beta30_state_only_R3_S_'+cid);gate=read(W/f'audit/review_{cid}.json');assert gate['passed'];met=read(W/f'results/metrics/{cid}.json')
  p,e,c,a,s,ad=[raw.load(run,n) for n in ['repair_plant_raw','repair_entry_diag','repair_cycle_diag','repair_actuator_command','asb_suspension','asb_diag']]
  t=p[:,0];n=len(t);ad=ad[:n];start=met['entry']['phase_start_s'];tau=t-start;ix=np.minimum(np.rint(t/DT).astype(int)//20,len(e)-1)
  x=np.c_[np.hypot(p[:,2],p[:,3])/3.6,p[:,4],np.deg2rad(p[:,5])];ref=e[ix,16:19].copy();ref[:,1]=np.rad2deg(ref[:,1]);m=(tau>=-1e-10)&(tau<8-1e-10);pulse=(tau>=1.5-1e-10)&(tau<2-1e-10)
  tv,pv=planar(run);vsm=(tv>=start-1e-10)&(tv<start+8-1e-10);vsp=(tv>=start+1.5-1e-10)&(tv<start+2-1e-10)
  z=x[m];rr=ref[m];fz=p[m,17:21];kk=p[m,27:31];fx=p[m,9:13];fy=p[m,13:17];tt=a[m,5:9];physical=np.c_[fz,kk,fx,fy,s[m,25:27],s[m,1:3],s[m,17:21]]
  if base is None:base={'x':x.copy(),'ref':ref.copy(),'p':p.copy(),'a':a.copy(),'s':s.copy(),'planar':pv.copy(),'tv':tv.copy(),'t':t.copy(),'start':start}
  assert start==base['start'] and np.array_equal(t,base['t']) and np.array_equal(tv,base['tv'])
  delta=x-base['x'];dfz=p[:,17:21]-base['p'][:,17:21];dp=pv-base['planar']
  same_ref=np.max(abs(ref[m]-base['ref'][m]),axis=0)
  own8=stat(z,rr) if m.sum()==16000 else None;shared8=stat(z,base['ref'][m]) if m.sum()==16000 else None
  ownpulse=stat(x[pulse],ref[pulse]) if pulse.sum()==1000 else None;sharedpulse=stat(x[pulse],base['ref'][pulse]) if pulse.sum()==1000 else None
  en=read(run/'used_config.json')['controller']['entry'];cm=(e[:,0]>=start-1e-10)&(e[:,0]<start+8-1e-10);ee=e[cm];req=ee[:,26:29];ff=ee[:,23:26];budget=ee[:,29:32]
  low=np.c_[ff[:,0]-en['steer_deviation'],np.full(len(ee),en['slip_low'][0]),np.full(len(ee),en['slip_low'][1])];high=np.c_[ff[:,0]+en['steer_deviation'],np.full(len(ee),en['slip_high'][0]),np.full(len(ee),np.inf)]
  amp=np.clip(req,low,high);ampclip=abs(req-amp)>1e-10;slewclip=abs(budget-amp)>1e-10
  assert abs(ee[:,44:48]+ee[:,48:52]+ee[:,52:56]-ee[:,56:60]).max()<1e-8
  fast=np.any(abs(a[m,3:5])>=np.rad2deg(en['steering_rate_rad_s'])-1e-7,axis=1)
  inputs={'amp_clip_s':(ampclip.sum(0)*.01).tolist(),'slew_clip_s':(slewclip.sum(0)*.01).tolist(),'request_budget_gap_peak':abs(req-budget).max(0).tolist(),'actual_rate_boundary':span(fast),'actual_rate_peak_deg_s':float(abs(a[m,3:5]).max()),'wheel_projection':span(e[ix[m],65]>.5),'torque_protection':span(e[ix[m],66]>.5),'torque_RMS_wheels':np.sqrt(np.mean(tt**2,axis=0)).tolist(),'torque_peak_wheels':abs(tt).max(0).tolist(),'torque_margin_Nm':(3000-abs(tt).max(0)).tolist()}
  ok=abs(z-TARGET)<=[.2,2,.03];ev=met['entry']['first600'];entry=ev['start_s']-start if ev else None;last=[]
  if ev:
   j=int(round((ev['start_s']-start)/DT));last=[['V','beta','AVz'][q]for q in np.flatnonzero(~ok[max(0,j-1)])]
  singles=[metrics.contiguous_first(tau[m],ok[:,k],600)for k in range(3)]
  d0=I.domain_window(run,start,start+8);dfull=DOM.audit(run,raw.load);domains.append({'case':cid,'entry8':d0,'full_run':dfull})
  rear=[r for r in d0['table_domains']if r['wheel']=='RR'and r['table'].startswith('FS_COMP_TABLE')][0]
  fminloc=np.unravel_index(fz.argmin(),fz.shape)
  power=ad[:,29:31];spower=ad[:,31:33];pulse_time=np.flatnonzero(pulse)
  mech={'moment_peak_Nm':abs(ad[:,7:9]).max(0).tolist(),'received_peak_Nm':abs(ad[:,9:11]).max(0).tolist(),'rate_peak_Nm_s':abs(ad[:,35:37]).max(0).tolist(),'abs_moment_integral_Nm_s':(abs(ad[:,7:9]).sum(0)*DT).tolist(),'roll_power_peak_W':abs(power[pulse]).max(0).tolist(),'roll_abs_work_J':(abs(power[pulse]).sum(0)*DT).tolist(),'spring_power_peak_W':abs(spower[pulse]).max(0).tolist(),'spring_abs_work_J':(abs(spower[pulse]).sum(0)*DT).tolist()}
  d={'case':cid,'source':str(run),'raw_sha256':gate['raw_sha256'],'entry_start_s':start,'entry_elapsed_s':entry,'entry_confirm_elapsed_s':ev['confirmed_s']-start if ev else None,'entry_singles':singles,'last_unmet':last,'metrics':met,'own8':own8,'shared8':shared8,'ownpulse':ownpulse,'sharedpulse':sharedpulse,'reference_max_difference8':same_ref.tolist(),'inputs':inputs,'mechanical_diagnostics':mech,'delta_pulse_peak_states':abs(delta[pulse]).max(0).tolist(),'delta_pulse_peak_Fz':abs(dfz[pulse]).max(0).tolist(),'delta_pulse_Mz_RMS':float(np.sqrt(np.mean(dp[vsp,-1]**2))),'planar_delta_RMS_components':np.sqrt(np.mean(dp[vsp,4:7]**2,axis=0)).tolist(),'full8':m.sum()==16000,'fullpulse':pulse.sum()==1000}
  details.append(d)
  row={'case':cid,'classification':'PASS' if met['passed'] else ('PARTIAL' if ev and met['hard_execution_validation_passed'] else 'FAIL'),'entry_s':entry,'entry_delta_ms':(entry-details[0]['entry_elapsed_s'])*1000 if entry is not None else None,'entry_confirm_s':d['entry_confirm_elapsed_s'],'last_unmet':'+'.join(last),'V_min_0_8':float(z[:,0].min()),'V_max_0_8':float(z[:,0].max()),'beta_min_0_8':float(z[:,1].min()),'AVz_peak_0_8':float(z[:,2].max()),'Fz_min_0_8':float(fz.min()),'Fz_min_wheel':WH[fminloc[1]],'Fz_min_tau_s':float(tau[m][fminloc[0]]),'torque_RMS_0_8':float(np.sqrt(np.mean(tt**2))),'torque_peak_0_8':float(abs(tt).max()),'steer_amplitude_clip_s':inputs['amp_clip_s'][0],'front_slip_clip_s':inputs['amp_clip_s'][1],'rear_slip_clip_s':inputs['amp_clip_s'][2],'steer_rate_boundary_s':inputs['actual_rate_boundary']['total_s'],'wheel_projection_s':inputs['wheel_projection']['total_s'],'torque_protection_s':inputs['torque_protection']['total_s'],'recovery_s':met['phase4']['first_straight2000']['elapsed_from_phase4_s'] if met['phase4']['first_straight2000'] else None,'full_run_minFz_N':float(p[:,17:21].min()),'RR_spring_excess_0_8_mm':rear['high']['max_excess'],'RR_spring_exposure_0_8_s':rear['high']['cumulative_left_hold_s']}
  for j,key in enumerate(['V','beta','AVz']):
   row['Jshared_pulse_'+key]=sharedpulse['J'][j] if sharedpulse else None;row['Jshared8_'+key]=shared8['J'][j] if shared8 else None;row['Jown8_'+key]=own8['J'][j] if own8 else None
   row['hold_RMSE_'+key]=met['phase3_hold']['state_rmse'][j] if met['phase3_hold'] else None
  rows.append(row)
  for delay in [.1,.2,.3,.4,.49]:
   j=int(round((start+1.5+delay)/DT));v=int(round((start+1.5+delay)/.01));assert abs(tv[v]-t[j])<1e-9
   r={'case':cid,'delay_s':delay,'tau_s':1.5+delay}
   for k,key in enumerate(['V','beta','AVz']):r['delta_'+key]=float(delta[j,k]);r['error_A_'+key]=float(base['x'][j,k]-base['ref'][j,k]);r['error_case_'+key]=float(x[j,k]-base['ref'][j,k])
   for k,wh in enumerate(WH):
    for key,col in [('Fz',17),('Fx_wheel',9),('Fy_wheel',13),('kappa',27)]:r['delta_'+key+'_'+wh]=float(p[j,col+k]-base['p'][j,col+k])
   for k,key in enumerate(['Fx_body_f','Fx_body_r','Fy_body_f','Fy_body_r','Mz_Fy_planar','Mz_Fx_planar','Mz_align_planar','Mz_sum_planar']):r['delta_'+key]=float(dp[v,k])
   samples.append(r)
  # 显示缓存仅抽取每10个原生点，关键脉冲图保持原生点；全部指标在抽样前计算。
  cache(cid+'_state',np.c_[tau[m],z,rr,delta[m]][::10]);cache(cid+'_physical',np.c_[tau[m],physical][::10]);cache(cid+'_actual',np.c_[tau[m],a[m,1:9]][::10]);cache(cid+'_control',np.c_[ee[:,0]-start,ff,req,budget,ampclip,slewclip,ee[:,44:64]])
  cache(cid+'_pulse',np.c_[tau[pulse],ad[pulse,7:11],dfz[pulse],delta[pulse],power[pulse],spower[pulse]])
  cache(cid+'_planar',np.c_[tv[vsm]-start,pv[vsm],dp[vsm]])
  cache(cid+'_trajectory',np.c_[t,p[:,35:37]][::20])
  cache(cid+'_fullstate',np.c_[t,x,ref][::20])
  _,sv,sa=DOM.vs_read(run)
  cache(cid+'_suspension',np.c_[tv[vsm]-start,sv[vsm,sa['FRSt_L1'][0]],sv[vsm,sa['FJSt_R2'][0]]])
 dump(OUT/'details.json',details);dump(OUT/'suspension_domains.json',domains);table(OUT/'summary.csv',rows);table(OUT/'same_delay_response.csv',samples)
 dump(OUT/'data_contract.json',{'native_dt_s':DT,'control_dt_s':.01,'VSB_dt_s':.01,'pulse_samples':1000,'entry_samples':16000,'power_is_proxy_not_electrical':True,'planar_yaw_moment_not_3D_net':True,'cache_rebuild':'internal/analyze_batch.py','vehicle_simulations_in_analysis':0})
 print(json.dumps(rows,ensure_ascii=False))
if __name__=='__main__':main()
