"""Step35n唯一离线入口：读取冻结Step35m原始记录，复算事件及代价；不加载模型或启动车辆。

状态/实收输入0.0005 s，控制裁剪0.01 s，止挡沿VS原时轴。仅写本目录的小表、报告素材与可重建绘图缓存。
"""
import sys, json, csv, hashlib, importlib.util
from pathlib import Path
import numpy as np
sys.dont_write_bytecode = True
W = Path(__file__).resolve().parents[1]
M = W.parent / 'step35m_synchronous_exit_20261009'
sys.path.insert(0, str(M/'internal'))
spec=importlib.util.spec_from_file_location('m_analysis',M/'internal/analyze_cases.py')
a=importlib.util.module_from_spec(spec);spec.loader.exec_module(a)
DT=.0005; CT=.01; WH=['FL','FR','RL','RR']
OUT=W/'results'; CACHE=W/'cache'; AUD=W/'audit'
for p in [OUT,CACHE,AUD]:p.mkdir(parents=True,exist_ok=True)
def read(p):return json.loads(Path(p).read_text(encoding='utf-8'))
def sha(p):
 h=hashlib.sha256()
 with Path(p).open('rb') as f:
  for b in iter(lambda:f.read(2**20),b''):h.update(b)
 return h.hexdigest()
def dump(p,v):Path(p).write_text(json.dumps(v,ensure_ascii=False,indent=2,allow_nan=False,default=lambda x:x.tolist() if isinstance(x,np.ndarray) else x.item()),encoding='utf-8')
def csvout(p,rows):
 if not rows:return
 with Path(p).open('w',encoding='utf-8-sig',newline='') as f:
  cw=csv.DictWriter(f,fieldnames=list(rows[0]));cw.writeheader();cw.writerows(rows)
def plotcache(name,header,z):np.savetxt(CACHE/name,z,delimiter=',',header=header,comments='',fmt='%.17g')
def event(mask,count,dt=DT):
 """累计和寻找首个合格段；独立于历史基于段边界的实现。"""
 b=np.asarray(mask,dtype=bool); cs=np.r_[0,np.cumsum(b,dtype=np.int64)]
 ii=np.flatnonzero(cs[count:]-cs[:-count]==count)
 if not len(ii):return None
 k=int(ii[0]);return {'start_s':k*dt,'confirmed_s':(k+count-1)*dt,'index':k,'confirmed_index':k+count-1}
def spans(mask,dt):
 b=np.asarray(mask,dtype=bool); edges=np.diff(np.r_[False,b,False].astype(int)); st=np.flatnonzero(edges==1);en=np.flatnonzero(edges==-1)
 return {'first_s':float(st[0]*dt) if len(st) else None,'at_start':bool(b[0]),'total_s':int(b.sum())*dt,'longest_s':int(np.max(en-st))*dt if len(st) else 0.,'segments':len(st)}
def stats(z):return {'mean':np.mean(z,axis=0),'std':np.std(z,axis=0),'peak_to_peak':np.ptp(z,axis=0),'min':np.min(z,axis=0),'max':np.max(z,axis=0)}
def errors(z,ref):
 e=z-ref
 return {'J':np.sum(e*e,axis=0)*DT,'RMSE':np.sqrt(np.mean(e*e,axis=0)),'peak_abs':np.max(abs(e),axis=0)}
def verify_manifest():
 records=read(M/'MANIFEST.json')['files']
 for f in records:assert sha(M/f['path'])==f['sha256'],f['path']
 return {'files':len(records),'manifest_sha256':sha(M/'MANIFEST.json'),'all_match':True}

def main():
 pre=verify_manifest();dump(AUD/'source_before.json',pre)
 hist=read(M/'results/analysis/comparison.json'); selected=[d for d in hist['details'] if d['row']['version']=='new'];assert len(selected)==5
 rows=[]; details=[]; events=[]; evidence=[]; checks=[]; base=None; sched0=None
 for d in selected:
  run=Path(d['source']);case=d['row']['configuration']; used=read(run/'used_config.json'); meta=read(run/'archive_index.json')
  assert sha(run/'raw_numeric.mat')==meta['archive_sha256']==d['raw_sha256']
  vc=used['vehicle_config']
  for key in ['source','simfile','origin_cfg']:
   assert sha(vc[key+'_path'])==vc[key+'_sha256'],key
  p,e,c,u,s,im=[a.raw.load(run,n) for n in ['repair_plant_raw','repair_entry_diag','repair_cycle_diag','repair_actuator_command','asb_suspension','asb_import12']]
  t=p[:,0];assert np.max(abs(np.diff(t)-DT))<1e-10 and len(t)==120001
  for ar in [u,s,im]:assert np.array_equal(ar[:,0],t)
  assert np.max(abs(np.diff(e[:,0])-CT))<1e-10
  assert np.array_equal(u,im[:,:9]) and np.all(im[:,9:13]==0)
  auxerr=float(np.max(abs(s[:,25:27]-s[:,1:3]*[vc['kf_deg'],vc['kr_deg']])));assert auxerr<1e-8
  start=float(c[c[:,1]==4][0,0]);i=int(round(start/DT));sl=slice(i,i+30000);tau=np.arange(30000)*DT
  assert i>=2000 and i+30000<=len(t)
  x=np.c_[np.hypot(p[:,2],p[:,3])/3.6,p[:,4],np.deg2rad(p[:,5])];z=x[sl]
  assert np.max(abs(x[:,1]-np.rad2deg(np.arctan2(p[:,3],p[:,2]))))<1e-9
  # 用原零阶保持索引；不插值状态或参考。
  ci=np.rint(t/DT).astype(int)//20;ci=np.minimum(ci,len(e)-1)
  ref=e[ci[sl],16:19].copy();ref[:,1]=np.rad2deg(ref[:,1])
  ok=abs(x[i:]-[12,0,0])<=[.2,1,.02];ev=event(ok.all(axis=1),2000);assert ev
  assert abs(ev['start_s']-d['row']['recovery_s'])<=DT+1e-12
  oldev=a.ha.event(t[i:]-start,ok.all(axis=1),2000);assert abs(oldev['start_s']-ev['start_s'])<1e-10
  singles=[event(ok[:,k],2000) for k in range(3)]
  last=[['V','beta','AVz'][k] for k in np.flatnonzero(~ok[ev['index']-1])]
  after=ev['confirmed_index']+1;post=spans(~ok[after:].all(axis=1),DT);assert after+2000<=len(ok)
  postx=x[i+after:i+after+2000];poststats=stats(postx);poststats['RMSE_final']=errors(postx,np.array([12,0,0]))['RMSE']
  own=errors(z,ref);target=errors(z,np.array([12,0,0]));fz=p[sl,17:21];torque=u[sl,5:9];cfg=used['controller']['entry']
  assert cfg['torque_limit']==3000 and cfg['slip_high'][1] is None
  cm=np.flatnonzero((e[:,0]>=start-1e-10)&(e[:,0]<start+15-1e-10));assert len(cm)==1500
  ee=e[cm];ct=np.arange(1500)*CT;n=a.series(used['profile'],4,ct)
  assert n['K'].shape==(1500,3,5)
  sched=np.c_[n['x'],n['v'],n['K'].reshape(1500,15),n['offset']]
  sched_error=0. if sched0 is None else float(np.max(abs(sched-sched0)))
  if sched0 is None:sched0=sched.copy()
  assert sched_error<1e-10
  assert np.max(abs(n['x']-ee[:,16:23]))<1e-10 and np.max(abs(n['v']-ee[:,23:26]))<1e-10
  terms=-n['K']*(ee[:,9:14]-n['x'][:,:5])[:,None,:];req=n['v']+terms.sum(axis=2)
  assert np.max(abs(req-ee[:,26:29]))<1e-9
  rates=np.array([cfg['steering_rate_rad_s'],cfg['slip_rate'],cfg['slip_rate']])
  lo=np.c_[n['v'][:,0]-cfg['steer_deviation'],np.full(1500,cfg['slip_low'][0]),np.full(1500,cfg['slip_low'][1])]
  hi=np.c_[n['v'][:,0]+cfg['steer_deviation'],np.full(1500,cfg['slip_high'][0]),np.full(1500,np.inf)]
  amp=np.clip(req,lo,hi);prev=e[cm-1,29:32];bud=ee[:,29:32]
  assert np.max(abs(bud-np.clip(amp,prev-CT*rates,prev+CT*rates)))<1e-9
  # 累计裁剪用既有容差；零余量、裁剪参与和实际硬违规分开。
  masks=[]
  def add(name,layer,mask,dt,margin,unit,request=None,budget=None,lower=None,upper=None):
   sp=spans(mask,dt); mm=np.asarray(margin); item={'case':case,'constraint':name,'layer':layer,'resolution_s':dt,**sp,'min_margin':float(np.min(mm)),'margin_unit':unit}
   events.append(item);masks.append((name,np.asarray(mask),dt))
   if sp['first_s'] is not None and request is not None:
    j=int(round(sp['first_s']/dt));inds=np.arange(max(0,j-1),min(len(mask),j+2))
    for k in inds:evidence.append({'case':case,'constraint':name,'tau_s':k*dt,'active':bool(mask[k]),'request':float(request[k]),'budget':float(budget[k]),'lower':float(lower[k]) if np.isfinite(lower[k]) else None,'upper':float(upper[k]) if np.isfinite(upper[k]) else None,'unit':unit})
   return item
  for k,name in enumerate(['steering','front_slip','rear_slip']):
   mar=np.minimum(bud[:,k]-lo[:,k],hi[:,k]-bud[:,k]);unit='rad' if k==0 else '1'
   add(name+'_amplitude_clip','execution',abs(req[:,k]-amp[:,k])>1e-10,CT,mar,unit,req[:,k],bud[:,k],lo[:,k],hi[:,k])
   add(name+'_target_slew','execution',abs(amp[:,k]-bud[:,k])>1e-10,CT,rates[k]-abs(bud[:,k]-prev[:,k])/CT,unit+'/s',amp[:,k],bud[:,k],prev[:,k]-CT*rates[k],prev[:,k]+CT*rates[k])
  actualrate=u[sl,3:5];rdeg=np.rad2deg(rates[0]);ratehit=np.any(abs(actualrate)>=rdeg-1e-7,axis=1)
  nativeprev=u[i-1:i+29999,1:3];nativetarget=np.repeat(np.rad2deg(e[ci[sl],29:30]),2,axis=1)
  assert np.max(abs(u[sl,1:3]-np.clip(nativetarget,nativeprev-rdeg*DT,nativeprev+rdeg*DT)))<1e-8
  assert np.max(abs(actualrate-(u[sl,1:3]-nativeprev)/DT))<1e-7
  rate_event=add('actual_steering_rate','execution',ratehit,DT,rdeg-np.max(abs(actualrate),axis=1),'deg/s')
  # Wheel budget is diagnosed at controller resolution; contact assertion also executes there.
  wd=ee[:,36:40];wdprev=e[cm-1,36:40]; accel=(wd-wdprev)/CT
  omega=ee[:,40:44];load=ee[:,52:56];kw=cfg['wheel_kw'];Iw=cfg['vehicle']['Iw'];gain=kw+Iw/CT
  wlo=np.maximum((-3000+kw*omega+Iw/CT*wdprev-load)/gain,wdprev-cfg['omega_acceleration']*CT)
  whi=np.minimum((3000+kw*omega+Iw/CT*wdprev-load)/gain,wdprev+cfg['omega_acceleration']*CT)
  assert np.all(wlo<=whi) and np.max(abs(wd-np.clip(ee[:,32:36],wlo,whi)))<1e-9
  assert np.max(abs(ee[:,56:60]-(kw*(wd-omega)+Iw*accel+load)))<1e-7
  assert np.max(abs(ee[:,60:64]-np.clip(ee[:,56:60],-3000,3000)))<1e-8
  wheel_margin=cfg['omega_acceleration']-np.max(abs(accel),axis=1)
  add('wheel_budget_projection','budget',ee[:,65]>.5,CT,wheel_margin,'rad/s2')
  add('empty_joint_wheel_budget','hard',np.any(wlo>whi,axis=1),CT,np.min(whi-wlo,axis=1),'rad/s')
  add('torque_protection','protection',ee[:,66]>.5,CT,3000-np.max(abs(ee[:,60:64]),axis=1),'N m')
  add('contact_velocity_nonpositive','hard',np.any(ee[:,67:71]<=0,axis=1),CT,np.min(ee[:,67:71],axis=1),'m/s')
  add('torque_budget_violation','hard',np.any(abs(torque)>3000+1e-9,axis=1),DT,3000-np.max(abs(torque),axis=1),'N m')
  add('nonpositive_normal_load','hard',np.any(fz<=0,axis=1),DT,np.min(fz,axis=1),'N')
  # 原生后簧查询包含静态偏置，直接保留既有查询定义；不把表端当机械行程。
  for k in [2,3]:
   cmp=s[sl,17+k];add('rear_spring_domain_'+WH[k],'model_domain',abs(cmp)>50,DT,50-abs(cmp),'mm')
  domains=a.oldana.domain_window(run,start,start+15)
  # 重新读原VS/VSB，止挡只按原时刻分析，不生成0.0005 s伪采样。
  tv,vv,aliases=a.oldana.domain.vs_read(run);vm=(tv>=start-1e-10)&(tv<start+15-1e-10);vt=tv[vm]-start
  vdt=float(np.median(np.diff(tv)));assert abs(vdt-.01)<1e-6
  stops=[]; stopdata=[]
  for typ in ['FJSt_','FRSt_']:
   for wheel,tail in zip(WH,['L1','R1','L2','R2']):
    v=vv[vm,aliases[typ+tail][0]];b=abs(v)>1e-6;sp=spans(b,vdt)
    if sp['first_s'] is not None:sp['first_s']=float(vt[np.flatnonzero(b)[0]])
    rec={'signal':typ+tail,'wheel':wheel,'peak_abs_N':float(abs(v).max()),'peak_tau_s':float(vt[np.argmax(abs(v))]),'participation_threshold_N':1e-6,'resolution_s':vdt,**sp};stops.append(rec);stopdata.append(v)
  idx=np.unravel_index(np.argmin(fz),fz.shape);row={'configuration':f"{vc['kf_deg']}/{vc['kr_deg']}",'case':case,'classification':d['row']['classification'],'recovery_s':ev['start_s'],'confirm_s':ev['confirmed_s'],'last_unmet':'+'.join(last),'speed_loss_mps':max(0.,12-float(z[:,0].min())),'overspeed_mps':max(0.,float(z[:,0].max())-12),'beta_positive_peak_deg':max(0.,float(z[:,1].max())),'AVz_reverse_peak_rad_s':max(0.,-float(z[:,2].min())),'torque_RMS_Nm':float(np.sqrt(np.mean(torque**2))),'torque_peak_Nm':float(abs(torque).max()),'torque_min_margin_Nm':float(3000-abs(torque).max()),'minFz_N':float(fz[idx]),'minFz_wheel':WH[idx[1]],'minFz_tau_s':idx[0]*DT,'steering_rate_first_s':rate_event['first_s'],'steering_rate_total_s':rate_event['total_s'],'steering_rate_longest_s':rate_event['longest_s'],'steering_rate_peak_deg_s':float(abs(actualrate).max()),'front_slip_clip_first_s':next(v['first_s'] for v in events if v['case']==case and v['constraint']=='front_slip_amplitude_clip'),'front_slip_clip_total_s':float(np.sum(abs(req[:,1]-amp[:,1])>1e-10)*CT),'wheel_projection_s':float(np.sum(ee[:,65]>.5)*CT),'torque_protection_s':float(np.sum(ee[:,66]>.5)*CT),'RR_spring_excess_mm':max(0.,float(abs(s[sl,20]).max())-50),'RR_spring_exposure_s':float(np.sum(abs(s[sl,20])>50)*DT),'reexit_after_confirm_s':post['total_s']}
  for k,nam in enumerate(['V','beta','AVz']):
   for prefix,val in [('J_final',target['J']),('J_own',own['J']),('RMSE_own',own['RMSE']),('peak_error_own',own['peak_abs']),('post1s_RMSE',poststats['RMSE_final']),('post1s_peak_to_peak',poststats['peak_to_peak'])]:row[prefix+'_'+nam]=float(val[k])
  for k,wheel in enumerate(WH):row['torque_RMS_'+wheel]=float(np.sqrt(np.mean(torque[:,k]**2)));row['torque_peak_'+wheel]=float(abs(torque[:,k]).max())
  for k,nam in enumerate(['steering','front_slip','rear_slip']):row[nam+'_amp_clip_peak']=float(abs(req[:,k]-amp[:,k]).max());row[nam+'_request_budget_gap_peak']=float(abs(req[:,k]-bud[:,k]).max())
  for key in ['recovery_s','torque_RMS_Nm','torque_peak_Nm','minFz_N']:assert abs(row[key]-d['row'][key])<1e-8,key
  for k,nam in enumerate(['V','beta','AVz']):
   assert abs(row['J_final_'+nam]-d['row']['Jtarget15_'+nam])<1e-7
   assert abs(row['J_own_'+nam]-d['row']['Jown15_'+nam])<1e-7
  assert row['classification']=='PASS' and not read(run/'summary.json')['error']
  if base is None:base=fz.copy()
  delta=fz-base;j=np.unravel_index(np.argmax(abs(delta)),delta.shape)
  detail={'row':row,'source':str(run),'raw_sha256':meta['archive_sha256'],'joint_event':ev,'single_events':singles,'post_confirmation_full_record':post,'post_confirmation_next_1s':poststats,'pre_exit_1s':{'state':stats(x[i-2000:i]),'Fz':stats(p[i-2000:i,17:21]),'Aux':stats(s[i-2000:i,25:27])},'same_time_Fz_delta':{'peak_abs_N':float(abs(delta).max()),'wheel':WH[j[1]],'tau_s':j[0]*DT,'RMS_per_wheel_N':np.sqrt(np.mean(delta**2,axis=0)),'at_own_min_delta_N':float(delta[idx]),'base_at_own_min_N':float(base[idx])},'joint_wheel_budget_min_distance_rad_s':float(np.minimum(wd-wlo,whi-wd).min()),'stops':stops,'domains':domains,'reference_equal_max_error':sched_error,'Aux_relation_max_error_Nm':auxerr,'no_vehicle_run':True}
  details.append(detail);rows.append(row);checks.append({'case':case,'raw_hash_match':True,'config_hash_match':True,'native_window':30000,'control_window':1500,'independent_event_match':True,'historical_metrics_match':True,'reference_equal_error':sched_error,'Aux_error':auxerr})
  # 缓存只服务绘图，指标已全部在原生分辨率上完成。绘图抽点不会反写原始。
  plotcache(case+'_states.csv','tau,V,beta,AVz,Vref,betaref,AVzref',np.c_[tau,z,ref][::20])
  plotcache(case+'_physical.csv','tau,FL,FR,RL,RR,kFL,kFR,kRL,kRR,FxFL,FxFR,FxRL,FxRR,FyFL,FyFR,FyRL,FyRR,AuxF,AuxR,CmpRL,CmpRR',np.c_[tau,fz,p[sl,27:31],p[sl,9:17],s[sl,25:27],s[sl,19:21]][::20])
  plotcache(case+'_constraints.csv','tau,steer_margin_deg,front_margin,rear_lower_margin,wheel_accel_margin,torque_margin,contact_min,front_clip,projection,protection',np.c_[ct,np.rad2deg(np.minimum(bud[:,0]-lo[:,0],hi[:,0]-bud[:,0])),np.minimum(bud[:,1]-lo[:,1],hi[:,1]-bud[:,1]),bud[:,2]-lo[:,2],wheel_margin,3000-np.max(abs(ee[:,60:64]),axis=1),np.min(ee[:,67:71],axis=1),abs(req[:,1]-amp[:,1])>1e-10,ee[:,65],ee[:,66]])
  plotcache(case+'_input.csv','tau,deltaFL,deltaFR,dFL,dFR,TFL,TFR,TRL,TRR',u[sl][::20,:9]-np.r_[start,np.zeros(8)])
  plotcache(case+'_stops.csv','tau,'+','.join(v['signal'] for v in stops),np.c_[vt,np.array(stopdata).T])
  # 时间线用原生布尔段的起终点，不抽点漏掉短时触限。
  segments=[]
  for name,b,dt in masks:
   edge=np.diff(np.r_[False,b,False].astype(int))
   for st,en in zip(np.flatnonzero(edge==1),np.flatnonzero(edge==-1)):segments.append({'constraint':name,'start_s':st*dt,'end_s':en*dt})
  dump(OUT/(case+'_events.json'),segments)
  print('VERIFIED',row['configuration'],'recover',ev['start_s'],'rate_first',rate_event['first_s'],'slip_first',row['front_slip_clip_first_s'],flush=True)
 csvout(OUT/'summary.csv',rows);csvout(OUT/'constraint_events.csv',events);csvout(AUD/'first_event_evidence.csv',evidence)
 dump(OUT/'details.json',details)
 # 对预先声明的主指标做分项比较，不赋权，不筛选有利子集。
 keys=['recovery_s','J_final_V','J_final_beta','J_final_AVz','speed_loss_mps','overspeed_mps','torque_RMS_Nm','torque_peak_Nm','steering_rate_total_s','front_slip_clip_total_s','minFz_N']
 values=np.array([[r[k]*(-1 if k=='minFz_N' else 1) for k in keys] for r in rows]);dominance=[]
 for i in range(5):
  for j in range(5):
   if i!=j and np.all(values[i]<=values[j]) and np.any(values[i]<values[j]):dominance.append([rows[i]['configuration'],rows[j]['configuration']])
 dump(OUT/'comparison_rules.json',{'dominance_metrics':keys,'minFz_direction':'maximize','other_directions':'minimize','unweighted_dominance_pairs':dominance,'no_statistical_or_robustness_claim':True,'no_failure_boundary_test':True})
 post=verify_manifest();assert post==pre;dump(AUD/'validation.json',{'passed':True,'vehicle_runs':0,'protected_manifest':post,'cases':checks,'constraint_tolerances':{'amplitude_slew':1e-10,'actual_rate_deg_s':1e-7,'stop_force_N':1e-6},'post_window':'2000 samples immediately after confirmation sample','hold_integral':'sum of 30000 left-held native samples times 0.0005 s'})
 print('STEP35N_ANALYSIS_COMPLETE',flush=True)
if __name__=='__main__':main()
