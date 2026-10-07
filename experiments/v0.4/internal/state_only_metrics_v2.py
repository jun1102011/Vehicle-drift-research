"""Step35c 无路径状态循环的原生数据验收。仅读取既有55通道raw，不启动车辆。"""
from __future__ import annotations
import argparse, csv, hashlib, importlib.util, json
from pathlib import Path
from typing import Any
import numpy as np

W = Path(__file__).resolve().parents[1]
LOADER = W.parent / "step33c_asb_amplitude_20261005/internal/data_archive.py"
DT_NATIVE, DT_CONTROL = .0005, .01
TARGET = np.array([12.0, -30.0, .5989797412715093]) # V m/s, beta deg, body AVz rad/s
DRIFT_TOL = np.array([.2, 2.0, .03])
STRAIGHT_TOL = np.array([.2, 1.0, .02])
ENTRY_N, HOLD_N, EXIT_N = 600, 20000, 2000

def sha(path: Path) -> str:
    h=hashlib.sha256()
    with path.open('rb') as f:
        for b in iter(lambda:f.read(1<<20), b''): h.update(b)
    return h.hexdigest()

def read_json(p: Path) -> dict[str,Any]:
    x=json.loads(p.read_text(encoding='utf-8'))
    if not isinstance(x,dict): raise ValueError(f'expected JSON object: {p}')
    return x

def load_reader():
    spec=importlib.util.spec_from_file_location('step35c_readonly_archive',LOADER)
    if spec is None or spec.loader is None: raise RuntimeError('Step33c archive reader unavailable')
    mod=importlib.util.module_from_spec(spec); spec.loader.exec_module(mod); return mod

def contiguous_first(t: np.ndarray, ok: np.ndarray, n: int):
    run=0
    for i,v in enumerate(ok):
        run=run+1 if v else 0
        if run==n:
            j=i-n+1
            return {'start_s':float(t[j]),'confirmed_s':float(t[i]),'first_last_span_s':float(t[i]-t[j]),'start_index':j}
    return None

def longest_samples(mask):
    best=cur=0
    for v in mask:
        cur=cur+1 if bool(v) else 0; best=max(best,cur)
    return best

def _matrix(raw,name,ncol):
    a=np.asarray(raw[name],dtype=float)
    if a.ndim!=2 or a.shape[1]!=ncol or not np.isfinite(a).all(): raise ValueError(f'{name}: expected finite Nx{ncol}')
    return a

def evaluate_arrays(raw:dict[str,np.ndarray], *, actual_end_s:float, plant_error:str|None=None, execution_hard:dict|None=None):
    """评价固定三事件；phase为cycle_diag原生控制列ZOH到0.0005s。"""
    p=_matrix(raw,'repair_plant_raw',51); c=_matrix(raw,'repair_cycle_diag',17)
    a=_matrix(raw,'repair_actuator_command',9); ed=_matrix(raw,'repair_entry_diag',71)
    susp=_matrix(raw,'asb_suspension',27); fb=np.asarray(raw['feedback_diag'],dtype=float)
    t=p[:,0]
    if len(t)<ENTRY_N or t[0]!=0 or np.max(abs(np.diff(t)-DT_NATIVE))>1e-10: raise ValueError('plant native time axis is not continuous 0.0005 s')
    if c.shape[0]<2 or c[0,0]!=0 or np.max(abs(np.diff(c[:,0])-DT_CONTROL))>1e-9: raise ValueError('cycle diagnostics must be 0.01 s')
    for name,x in [('actuator',a),('suspension',susp)]:
        if len(x)!=len(t) or not np.array_equal(x[:,0],t): raise ValueError(f'{name} not aligned with native plant time')
    if not np.array_equal(ed[:,0],c[:,0]): raise ValueError('entry diagnostics not aligned with control time')
    if fb.ndim!=2 or fb.shape[1]!=18 or not np.isfinite(fb).all() or not np.array_equal(fb[:,0],t): raise ValueError('feedback_diag must be the 17-item native event log aligned to plant time')
    # State columns match frozen Step35b native plant convention.
    v=np.hypot(p[:,2],p[:,3])/3.6; beta=p[:,4]; r=np.deg2rad(p[:,5])
    state=np.c_[v,beta,r]
    fidx=np.clip(np.searchsorted(c[:,0],t,side='right')-1,0,len(c)-1)
    # cycle_diag: time + [phase, exitProgress, substage, holdTime, gate, inside,
    # straightCapture, weight, exitElapsed, 7 rate diagnostics].
    phase=c[fidx,1].astype(int)
    entry_idx=np.flatnonzero(phase==1); hold_idx=np.flatnonzero(phase==3); exit_idx=np.flatnonzero(phase==4)
    if not len(entry_idx): entry_start=None
    else: entry_start=float(t[entry_idx[0]])
    if not len(hold_idx): hold_start=None
    else: hold_start=float(t[hold_idx[0]])
    if not len(exit_idx): exit_start=None
    else: exit_start=float(t[exit_idx[0]])
    drift_ok=np.all(abs(state-TARGET)<=DRIFT_TOL,axis=1) & (r>0)
    entry_end=hold_start if hold_start is not None else float(t[-1]+DT_NATIVE)
    em=(t>=entry_start)&(t<entry_end) if entry_start is not None else np.zeros(len(t),bool)
    entry=contiguous_first(t[em],drift_ok[em],ENTRY_N)
    before_hold=(fb[:,0]<=hold_start+1e-10) if hold_start is not None else np.ones(len(fb),bool)
    native_rows=fb[before_hold]
    confirm_rows=native_rows[(native_rows[:,5]>=0)&((hold_start is None)|(native_rows[:,5]<(hold_start+1e-10 if hold_start is not None else np.inf)))]
    native_capture={'schema_channel':'feedback_diag_existing','new_top_level_channel':False,
      'count_peak':float(np.max(native_rows[:,1])) if len(native_rows) else None,
      'latched_any':bool(np.any(native_rows[:,3]>0)) if len(native_rows) else False,
      'confirm_any':bool(len(confirm_rows)>0),
      'first_confirm_s':float(np.min(confirm_rows[:,5])) if len(confirm_rows) else None,
      'index_peak':float(np.max(native_rows[:,6])) if len(native_rows) else None}
    native_capture_ok=bool(entry and native_capture['confirm_any'] and native_capture['latched_any'] and native_capture['count_peak'] is not None and native_capture['count_peak']>=ENTRY_N)
    # Fixed window is phase3's first sample through exactly 20,000 native samples.
    hold=None; hold_rmse=None; hold_metrics={}; hold_failure_time=None
    if hold_start is not None:
        i=int(np.flatnonzero(t>=hold_start-1e-12)[0]); j=i+HOLD_N
        expected_exit=hold_start+10.0
        phase4_matches=exit_start is not None and abs(exit_start-expected_exit)<=1e-9
        control_hold=(c[:,0]>=hold_start-1e-10)&(c[:,0]<expected_exit-1e-10)
        phases_are_hold=bool(np.any(control_hold) and np.all(c[control_hold,1].astype(int)==3))
        if j<=len(t) and np.max(abs(np.diff(t[i:j])-DT_NATIVE))<1e-10 and phase4_matches and phases_are_hold:
            hs=state[i:j]; hold_rmse=np.sqrt(np.mean((hs-TARGET)**2,axis=0))
            outband=np.any(abs(hs-TARGET)>DRIFT_TOL,axis=1)
            # Keep mean beta and peak separately to expose transient excursions.
            hold_metrics={'samples':HOLD_N,'start_s':float(t[i]),'end_exclusive_s':float(t[i]+HOLD_N*DT_NATIVE),
                          'state_rmse':[float(x) for x in hold_rmse],
                          'beta_mean_deg':float(np.mean(hs[:,1])),'beta_abs_peak_deg':float(np.max(abs(hs[:,1]))),
                          'beta_peak_deviation_deg':float(np.max(abs(hs[:,1]-TARGET[1]))),
                          'state_peak_abs_error':[float(x) for x in np.max(abs(hs-TARGET),axis=0)],
                          'out_of_band_samples':int(np.sum(outband)),'out_of_band_cumulative_s':float(np.sum(outband)*DT_NATIVE),
                          'out_of_band_longest_s':float(longest_samples(outband)*DT_NATIVE),
                          'phase4_starts_exactly_after_fixed_window':phase4_matches,'all_control_samples_are_phase3':phases_are_hold}
            protect_torque=np.any(abs(a[i:j,5:9])>=3000-1e-6,axis=1)
            # Diagnostics 64:67 retain the existing request/projection/protection flags.
            ci=np.clip(np.searchsorted(c[:,0],t[i:j],side='right')-1,0,len(c)-1)
            ei=np.clip(np.searchsorted(ed[:,0],t[i:j],side='right')-1,0,len(ed)-1)
            protection_flag=ed[ei,66]>0
            # Frozen Step35b schema: entry_diag[:,66] is torque-protection active;
            # actual native torque threshold is retained as an independent consistency check.
            combined=protection_flag
            ratio=float(np.mean(combined)); longest=float(longest_samples(combined)*DT_NATIVE)
            fz=p[i:j,17:21]
            hold_metrics.update({'torque_protection_fraction':ratio,'torque_protection_longest_s':longest,
                                 'torque_threshold_samples':int(protect_torque.sum()),'protection_flag_samples':int(protection_flag.sum()),
                                 'protection_flag_matches_actual_torque_threshold':bool(np.array_equal(protect_torque,protection_flag)),
                                 'min_wheel_load_N':float(np.min(fz)),'all_wheel_loads_positive':bool(np.all(fz>0))})
            yaw=np.unwrap(np.deg2rad(p[i:j,37]))
            hold_metrics['native_yaw_rad_start_end']=[float(yaw[0]),float(yaw[-1])]
            hold={'passed':bool(np.all(hold_rmse<=DRIFT_TOL) and ratio<=.05 and longest<=.2 and np.all(fz>0) and np.all(abs(a[i:j,5:9])<=3000+1e-6) and hold_metrics['protection_flag_matches_actual_torque_threshold']),**hold_metrics}
            if not hold['passed']:
                bad=np.any(abs(hs-TARGET)>DRIFT_TOL,axis=1)|combined|protect_torque|np.any(fz<=0,axis=1)
                if np.any(bad): hold_failure_time=float(t[i+np.flatnonzero(bad)[0]])
        else:
            hold={'passed':False,'reason':'fixed_phase3_10s_window_not_available','available_samples':max(0,len(t)-i)}
            hold_failure_time=float(t[-1])
    # Recovery search starts at actual phase4 onset; it is independent of planned spatial straight.
    recovery=None
    if exit_start is not None:
        take=np.flatnonzero(t>=exit_start-1e-12)
        straight_ok=np.all(abs(state[take]-np.array([TARGET[0],0.,0.]))<=STRAIGHT_TOL,axis=1)
        recovery=contiguous_first(t[take],straight_ok,EXIT_N)
        if recovery: recovery['elapsed_from_phase4_s']=recovery['start_s']-exit_start
    fz_all=p[:,17:21]
    torque=a[:,5:9]
    cycle_rates=c[:,10:17]
    hard={'archive_verified':None,'external_asb_zero':None,'interface_verified':None,'source_freeze_verified':None,
          'vehicle_completed':bool(float(actual_end_s)>=60.0-1e-8),'plant_error_clear':not bool(plant_error)}
    if execution_hard: hard.update(execution_hard)
    hard.update({'all_wheel_loads_positive':bool(np.all(fz_all>0)),'all_wheel_torques_within_budget':bool(np.all(abs(torque)<=3000+1e-6))})
    hard_ok=all(v is True for k,v in hard.items() if k!='plant_error_clear') and hard['plant_error_clear'] and native_capture_ok
    reasons=[]
    if plant_error: reasons.append('plant_error')
    if entry_start is None: reasons.append('entry_phase_missing')
    if entry is None: reasons.append('entry_600_not_reached')
    if not native_capture_ok: reasons.append('native_capture_confirmation_missing')
    if hold is None or not hold.get('passed'): reasons.append('phase3_fixed_window_failed_or_missing')
    if exit_start is None: reasons.append('phase4_missing')
    if recovery is None: reasons.append('phase4_2000_straight_not_reached')
    if not hard_ok: reasons.append('execution_hard_validation_failed_or_missing')
    passed=not reasons
    classification='simerror' if plant_error else ('failentry' if entry is None else ('PASS' if passed else 'partialentry'))
    # Natural trajectory diagnostics only. No path-reference error or fitted radius gate.
    xy=p[:,35:37]; dxy=np.gradient(xy,t,axis=0); course=np.unwrap(np.arctan2(dxy[:,1],dxy[:,0]))
    yaw_native=np.unwrap(np.deg2rad(p[:,37]))
    body_avz_integral=np.r_[0.,np.cumsum(.5*(r[1:]+r[:-1])*np.diff(t))]
    fit_r=None
    if hold_start is not None and hold_metrics.get('samples')==HOLD_N:
        seg=xy[i:j]; q=seg-seg.mean(axis=0)
        if len(seg)>2 and np.linalg.matrix_rank(q)>=2:
            z=np.linalg.lstsq(np.c_[2*q,np.ones(len(q))],np.sum(q*q,axis=1),rcond=None)[0]
            rr=z[2]+z[0]**2+z[1]**2
            fit_r=float(np.sqrt(rr)) if rr>0 else None
    failure_candidates=[]
    if entry is None and entry_start is not None: failure_candidates.append(entry_end)
    if hold is not None and not hold.get('passed'): failure_candidates.append(hold_metrics.get('end_exclusive_s',float(t[-1])))
    if exit_start is None and not plant_error: failure_candidates.append(float(t[-1]))
    if exit_start is not None and recovery is None: failure_candidates.append(float(t[-1]))
    # 硬约束按实际首次越界时刻；记录/接口失败只在结束审核时可判定，不能误填t=0。
    if not hard_ok:
        for bad in [np.any(fz_all<=0,axis=1),np.any(abs(torque)>3000+1e-6,axis=1)]:
            hits=np.flatnonzero(bad)
            if len(hits): failure_candidates.append(float(t[hits[0]]))
        failure_candidates.append(float(t[-1]))
    first_failure=min(failure_candidates) if failure_candidates else (float(t[-1]) if not passed else None)
    return {'schema':'step35c-state-only-metrics-v1','passed':passed,'classification':classification,
      'failure_reasons':reasons,'first_failure_time_s':first_failure,'phase3_first_violation_time_s':hold_failure_time,
      'actual_end_s':float(actual_end_s),'target_V_beta_deg_bodyAVz_rad_s':TARGET.tolist(),'native_dt_s':DT_NATIVE,
      'entry':{'phase_start_s':entry_start,'first600':entry,'first600_completed_before_phase3':bool(entry and entry['confirmed_s']<entry_end),'native_capture':native_capture,'native_capture_gate_passed':native_capture_ok,
       'interpretation':'latched event demonstrates a prior 600-point in-band segment; it does not assert continuous in-band state until phase3.'},
      'phase3_hold':hold,'phase4':{'actual_start_s':exit_start,'first_straight2000':recovery,'search_started_at_actual_phase4':True},
      'analysis_source_sha256':sha(Path(__file__)), 'analysis_repair':'native feedback log time axis; full-run wheel load/torque checks; invalid-summary handling',
      'hard_execution_validation':hard,'hard_execution_validation_passed':hard_ok,
      'execution_diagnostics':{'max_abs_wheel_torque_Nm':float(np.max(abs(torque))),'min_wheel_load_N':float(np.min(fz_all)),
       'nonpositive_wheel_load_samples':int(np.sum(fz_all<=0)),'feedback_diag_shape':list(fb.shape),
       'cycle_diag_reference_rate_abs_peak_7':np.max(abs(cycle_rates),axis=0).tolist()},
      'natural_motion_diagnostics':{'xy_start_end_m':[[float(x) for x in xy[0]],[float(x) for x in xy[-1]]],
       'course_rad_unwrapped':[float(course[0]),float(course[-1])],'native_yaw_rad_unwrapped_start_end':[float(yaw_native[0]),float(yaw_native[-1])],
       'body_AVz_time_integral_rad_start_end':[float(body_avz_integral[0]),float(body_avz_integral[-1])],
       'psi_from_course_minus_beta_rad_start_end':[float(course[0]-np.deg2rad(beta[0])),float(course[-1]-np.deg2rad(beta[-1]))],
       'fitted_radius_phase3_m_diagnostic_only':fit_r,'path_metric_gate':False},
      'scope':'Native state-cycle response in the current CarSim parameter model; not a path-following or mechanical safety certification.'}

def evaluate_run(run:Path, read_channel):
    case=read_json(run/'case.json'); identity=read_json(run/'source_identity.json')
    profile_id=str(case['profile_id']); profile_path=W/'source/profiles'/f'{profile_id}.json'
    if not profile_path.is_file() or Path(identity['profile_path']).resolve()!=profile_path.resolve(): raise ValueError('profile identity path mismatch')
    profile_sha=sha(profile_path)
    if profile_sha!=identity.get('profile_sha256'): raise ValueError('profile identity SHA mismatch')
    meta=read_json(run/'archive_index.json')
    if meta.get('channel_count')!=55 or meta.get('bitwise_readback') is not True: raise ValueError('55-channel archive verification missing')
    rawpath=run/'raw_numeric.mat'
    if sha(rawpath)!=meta.get('archive_sha256'): raise ValueError('raw archive SHA mismatch')
    summary=read_json(run/'summary.json'); error=summary.get('error')
    names=('repair_plant_raw','repair_cycle_diag','repair_actuator_command','repair_entry_diag','asb_suspension','feedback_diag','asb_import12')
    raw={n:read_channel(run,n) for n in names}
    imp=raw['asb_import12']
    external_zero=bool(imp.ndim==2 and imp.shape[1]==13 and np.all(imp[:,9:13]==0))
    hard={}
    completion=run/'completion.json'
    if completion.exists():
        c=read_json(completion); hard={'archive_verified':c.get('archive_verified') is True,
          'interface_verified':c.get('interface_verified') is True,
          'source_freeze_verified':c.get('source_freeze_verified') is True,
          'external_asb_zero':external_zero and c.get('external_asb_zero') is True,
          'vehicle_completed':c.get('vehicle_completed') is True and float(summary.get('actual_end',0))>=60-1e-8}
    else: hard['external_asb_zero']=external_zero
    result=evaluate_arrays(raw,actual_end_s=float(summary.get('actual_end',raw['repair_plant_raw'][-1,0])),plant_error=error,execution_hard=hard)
    result.update(run_id=run.name,profile_id=profile_id,profile_sha256=profile_sha,
      source_identity_sha256=sha(run/'source_identity.json'),raw_sha256=sha(rawpath),archive_index_sha256=sha(run/'archive_index.json'),offline_only=True)
    return result

def main():
    ap=argparse.ArgumentParser(description=__doc__); ap.add_argument('--run',type=Path,action='append',required=True); ap.add_argument('--out',type=Path,required=True)
    z=ap.parse_args()
    if z.out.exists(): raise FileExistsError(f'refuse overwrite: {z.out}')
    reader=load_reader(); items=[]
    for x in z.run:
      try: items.append(evaluate_run(x.resolve(),reader.load))
      except Exception as exc:
       items.append({'schema':'step35c-state-only-metrics-v1','run_id':x.name,'classification':'invalid','passed':False,
        'failure_reasons':['invalid_or_incomplete_run_archive'],'invalid_reason':str(exc),'first_failure_time_s':None,'raw_sha256':None})
    z.out.mkdir(parents=True)
    for item in items: (z.out/f"{item['run_id']}.json").write_text(json.dumps(item,ensure_ascii=False,indent=2,allow_nan=False)+'\n',encoding='utf-8')
    cols=['run_id','classification','passed','entry_start_s','entry_confirmed_s','phase3_pass','phase3_V_rmse','phase3_beta_rmse_deg','phase3_AVz_rmse','phase3_beta_mean_deg','phase3_beta_peak_deviation_deg','protection_fraction','protection_longest_s','phase4_start_s','recovery_start_s','first_failure_time_s','raw_sha256']
    with (z.out/'summary.csv').open('w',encoding='utf-8-sig',newline='') as f:
      w=csv.DictWriter(f,fieldnames=cols); w.writeheader()
      for x in items:
       h=x.get('phase3_hold') or {}; rm=h.get('state_rmse') or [None]*3; en=x.get('entry',{}).get('first600') or {}; ex=x.get('phase4',{}).get('first_straight2000') or {}
       w.writerow(dict(run_id=x['run_id'],classification=x['classification'],passed=x['passed'],entry_start_s=en.get('start_s'),entry_confirmed_s=en.get('confirmed_s'),phase3_pass=h.get('passed'),phase3_V_rmse=rm[0],phase3_beta_rmse_deg=rm[1],phase3_AVz_rmse=rm[2],phase3_beta_mean_deg=h.get('beta_mean_deg'),phase3_beta_peak_deviation_deg=h.get('beta_peak_deviation_deg'),protection_fraction=h.get('torque_protection_fraction'),protection_longest_s=h.get('torque_protection_longest_s'),phase4_start_s=x.get('phase4',{}).get('actual_start_s'),recovery_start_s=ex.get('start_s'),first_failure_time_s=x.get('first_failure_time_s'),raw_sha256=x.get('raw_sha256')))
    print(json.dumps({'out':str(z.out),'runs':[x['run_id'] for x in items],'classification':[x['classification'] for x in items]},ensure_ascii=False))
if __name__=='__main__': main()
