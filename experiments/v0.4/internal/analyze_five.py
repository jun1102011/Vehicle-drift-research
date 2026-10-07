"""离线比较旧/新384基线；不写历史或重复原始时序，不调用仿真。"""
import csv,json,hashlib,importlib.util,sys
from pathlib import Path
import numpy as np
sys.dont_write_bytecode=True
W=Path(__file__).resolve().parents[1];D=W.parent/'step35d_entry_acceleration_20261007';OUT=W/'results/analysis';OUT.mkdir(parents=True,exist_ok=True)
ID='V12_beta30_state_only_R3_kf384_kr384';dt=.0005
def module(path,name):
 s=importlib.util.spec_from_file_location(name,path);m=importlib.util.module_from_spec(s);s.loader.exec_module(m);return m
def read(p):return json.loads(p.read_text(encoding='utf-8-sig'))
def sha(p):return hashlib.sha256(p.read_bytes()).hexdigest()
def blocks(mask):
 e=np.diff(np.r_[False,mask,False].astype(int));return list(zip(np.flatnonzero(e==1),np.flatnonzero(e==-1)))
reader=module(W.parent/'step33c_asb_amplitude_20261005/internal/data_archive.py','reader');metric=module(W/'internal/state_only_metrics_v2.py','metric')
rows=[];details=[];loaded=[]
cases=['kf384_kr384','kf192_kr384','kf576_kr384','kf384_kr192','kf384_kr576']
F=W.parent/'step35f_continuous_entry_20261007'
for case in cases:
 label=case.replace('kf','').replace('_kr','/')
 root=F if case=='kf384_kr384' else W
 ID='V12_beta30_state_only_R3_'+case
 run=root/'results/runs'/ID;metric.W=root;m=metric.evaluate_run(run,reader.load)
 p,e,a=[reader.load(run,n) for n in ['repair_plant_raw','repair_entry_diag','repair_actuator_command']]
 t=p[:,0];x=np.c_[np.hypot(p[:,2],p[:,3])/3.6,p[:,4],np.deg2rad(p[:,5])];start=m['entry']['phase_start_s'];i=round(start/dt);n=16000;en=m['entry']['first600'];h=m['phase3_hold'];ex=m['phase4']['first_straight2000']
 assert len(t)>i+n and abs(t[i]-start)<1e-9
 z=x[i:i+n];take=slice(i,i+n);ci=np.minimum(np.rint(t/dt).astype(int)//20,len(e)-1);ref=e[ci,16:19].copy();ref[:,1]=np.rad2deg(ref[:,1]);err=z-ref[take]
 picks=i+np.arange(0,n+1,20);b=x[picks,1];rates=np.diff(b)/.01;mid=(b[:-1]+b[1:])/2;slow=(mid>=-28)&(mid<=-5)&(abs(rates)<2)
 cm=(e[:,0]>=start-1e-9)&(e[:,0]<start+8-1e-9);cc=np.flatnonzero(cm);rq=e[cc,26:29];ff=e[cc,23:26];lim=e[cc,29:32]
 low=np.c_[ff[:,0]-.15,np.full(len(cc),-.1),np.full(len(cc),-.1)];high=np.c_[ff[:,0]+.15,np.full(len(cc),.05),np.full(len(cc),np.inf)];amp=np.clip(rq,low,high)
 slew=np.array([.9563444369840329,.5,.5])*.01;rec=np.clip(amp,e[cc-1,29:32]-slew,e[cc-1,29:32]+slew);assert np.max(abs(rec-lim))<1e-10
 row={'case':label,'classification':m['classification'],'entry_elapsed_s':en['start_s']-start if en else None,'entry_confirm_relative_s':en['confirmed_s']-start if en else None,
 'beta_initial_positive_peak_deg':float(z[:,1].max()),'beta_min_0_8_deg':float(z[:,1].min()),'beta_rate_peak_abs_deg_s':float(max(abs(rates))),
 'beta_slowing_5_to28_under2deg_s_total_s':float(slow.sum()*.01),'beta_slowing_longest_s':max((b-a for a,b in blocks(slow)),default=0)*.01,
 'J_beta_deg2s_0_8':float(np.sum(err[:,1]**2)*dt),'J_V_0_8':float(np.sum(err[:,0]**2)*dt),'J_AVz_0_8':float(np.sum(err[:,2]**2)*dt),
 'V_min_0_8':float(z[:,0].min()),'AVz_max_0_8':float(z[:,2].max()),'Fz_min_0_8':float(p[take,17:21].min()),'Fz_min_full':float(p[:,17:21].min()),
 'steer_amplitude_clipping_s':float(np.sum(abs(rq[:,0]-amp[:,0])>1e-9)*.01),'steer_request_budget_gap_peak_deg':float(np.rad2deg(np.max(abs(rq[:,0]-lim[:,0])))),
 'actual_steer_rate_bound_s':float(np.sum(np.any(abs(a[take,3:5])>=54.7945-1e-5,axis=1))*dt),
 'rear_slip_rate_bound_s':float(np.sum(abs(np.diff(e[:,31])[cc-1]/.01)>=.5-1e-7)*.01),
 'torque_peak_0_8_Nm':float(np.max(abs(a[take,5:9]))),'torque_rms_0_8_Nm':float(np.sqrt(np.mean(a[take,5:9]**2))),
 'wheel_budget_projection_s':float(np.sum(e[cm,65]>.5)*.01),'torque_protection_s':float(np.sum(e[cm,66]>.5)*.01),
 'hold_V_rmse':h['state_rmse'][0] if h and 'state_rmse' in h else None,'hold_beta_rmse_deg':h['state_rmse'][1] if h and 'state_rmse' in h else None,'hold_AVz_rmse':h['state_rmse'][2] if h and 'state_rmse' in h else None,
 'recovery_elapsed_s':ex['elapsed_from_phase4_s'] if ex else None,'failure_reasons':';'.join(m['failure_reasons'])}
 for angle in [-5,-10,-15,-20,-25,-28]:
  idx=np.flatnonzero(z[:,1]<=angle);row[f'beta_first_{abs(angle)}_s']=float(idx[0]*dt) if len(idx) else None
 row['beta_5_to28_s']=row['beta_first_28_s']-row['beta_first_5_s'] if row['beta_first_28_s'] is not None else None
 rows.append(row);details.append({'source':str(run),'raw_sha256':sha(run/'raw_numeric.mat'),'metrics':m,'row':row});loaded.append((p,e,a))

# 所有组参考必须一致；实际控制输入可随反馈不同，不能要求它们一致。
base_e=loaded[0][1];base_start=details[0]['metrics']['entry']['phase_start_s'];checks=[]
for j,(case,detail,arrays) in enumerate(zip(cases,details,loaded)):
 p,e,a=arrays;start=detail['metrics']['entry']['phase_start_s']
 take=(e[:,0]>=start-1e-9)&(e[:,0]<start+8-1e-9);ref_error=float(np.max(abs(e[take,16:26]-base_e[(base_e[:,0]>=base_start-1e-9)&(base_e[:,0]<base_start+8-1e-9),16:26])))
 assert ref_error<1e-10
 root=F if j==0 else W;run=Path(detail['source']);ind=read(root/'audit'/('independent_'+run.name+'.json'))
 assert ind['protocol_PASS']==detail['metrics']['passed']
 if detail['metrics']['entry']['first600']:
  assert abs(ind['entry_first600']['start_s']-detail['metrics']['entry']['first600']['start_s'])<1e-10
 if j:
  cfg=read(W/'source/file_cases'/case/'aux_case_cfg.json');old=Path(cfg['origin_source_path']).read_bytes();new=Path(cfg['source_path']).read_bytes();assert old==new
 checks.append({'configuration':case,'common_reference_max_abs':ref_error,'independent_metrics_agree':True})
# 与同配置旧2.5s分段结果比较；旧记录不复制。
oldrows=list(csv.DictReader((D/'results/final/summary.csv').open(encoding='utf-8-sig')))
for row in rows:
 old=next(x for x in oldrows if float(x['entry_reference_s'])==2.5 and x['configuration']==row['case'])
 row['old_segmented_entry_elapsed_s']=float(old['entry_elapsed_s'])
 row['entry_change_from_segmented_ms']=(row['entry_elapsed_s']-float(old['entry_elapsed_s']))*1000 if row['entry_elapsed_s'] is not None else None
 row['old_segmented_V_min']=float(old['V_min_0_8'])
 # Step35d旧表未保存幅值裁剪列，直接从同组旧raw复算，不补造数值。
 cfg='kf'+row['case'].replace('/','_kr');oldrun=D/'results/runs'/('V12_beta30_state_only_R3_'+cfg)
 oe=reader.load(oldrun,'repair_entry_diag');ot=float(old['entry_phase_start_s']);sel=(oe[:,0]>=ot-1e-9)&(oe[:,0]<ot+8-1e-9)
 row['old_segmented_steer_amplitude_clipping_s']=float(np.sum(abs(oe[sel,26]-oe[sel,23])>.15+1e-9)*.01)
states=[]
for detail,(p,e,a) in zip(details,loaded):
 i=round(detail['metrics']['entry']['phase_start_s']/dt);states.append(np.c_[np.hypot(p[i:i+16000,2],p[i:i+16000,3])/3.6,p[i:i+16000,4],np.deg2rad(p[i:i+16000,5])])
spread=np.ptp(np.stack(states),axis=0)
out={'rows':rows,'records':details,'checks':checks,'five_configuration_spread':{'V_beta_AVz_same_time_range_peak':np.max(spread,axis=0).tolist(),'V_beta_AVz_same_time_range_rms':np.sqrt(np.mean(spread**2,axis=0)).tolist(),'entry_elapsed_range_s':float(np.ptp([r['entry_elapsed_s'] for r in rows])) if all(r['entry_elapsed_s'] is not None for r in rows) else None},'native_window':'entry-relative [0,8), 16000 samples; reference own, target fixed','baseline_referenced_not_copied':str(F/'results/runs/V12_beta30_state_only_R3_kf384_kr384/raw_numeric.mat')}
(OUT/'comparison.json').write_text(json.dumps(out,ensure_ascii=False,indent=2,allow_nan=False),encoding='utf-8')
with (OUT/'summary.csv').open('w',encoding='utf-8-sig',newline='') as f:
 wr=csv.DictWriter(f,fieldnames=rows[0].keys());wr.writeheader();wr.writerows(rows)
print(json.dumps({'rows':rows,'spread':out['five_configuration_spread']},ensure_ascii=False))
