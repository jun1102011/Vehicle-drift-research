"""核对实际Echo和历史保护源；输出道路修复后每组证据汇总，不改原始数组。"""
from prepare_research import *
import numpy as np
import importlib.util
def main():
 spec=importlib.util.spec_from_file_location('road_domain',W.parent/'step35g_continuous_stiffness_20261007/internal/suspension_domain_audit.py');domain=importlib.util.module_from_spec(spec);spec.loader.exec_module(domain)
 freeze=read(W/'source/runner_freeze.json')
 for kind in ['files','external_files']:
  for x in freeze[kind]:assert sha(Path(x['path']) if kind=='external_files' else W/x['path'])==x['sha256'],x['path']
 rows=[]
 for c in CASES:
  run=W/f'results/runs/LeftRight_R3_{c}';idx=read(run/'archive_index.json');assert sha(run/'raw_numeric.mat')==idx['archive_sha256']
  echo=(run/'LastRun_echo.par.evidence').read_text(encoding='utf-8',errors='replace')
  keys={'OPT_PATH_LOOP(1)':'0','SEGMENT_TYPE(1,1)':'0','SEGMENT_LENGTH(1,1)':'2000','Y_PATH_START(1)':'-16','ROAD_ZS_CONSTANT(1)':'0','MU_ROAD_CONSTANT(1)':'0.85','MU_ROAD_L_CONSTANT(1)':'0','RR_SURF(1)':'1','OPT_INIT_PATH':'0','OPT_DM':'0'}
  for k,v in keys.items():
   a=re.findall(r'^'+re.escape(k)+r'\s+(\S+)',echo,re.M);assert a==[v],(c,k,a)
  log=(run/'LastRun_log.txt.evidence').read_text(encoding='utf-8',errors='replace')
  assert 'S and L values for path could not be calculated' not in log
  m=read(W/f'audit/review_{c}.json');reg=read(W/f'audit/road_regression_{c}.json')
  tv,v,aliases=domain.vs_read(run);ground={};boundary={}
  for wheel in ['L1','R1','L2','R2']:
   for prefix,value in [('Zgnd_',0),('dZdX_',0),('dZdY_',0),('MuX_',.85),('MuY_',.85)]:
    key=prefix+wheel;column,unit=aliases[key];yy=v[:,column].astype(float);err=float(abs(yy-value).max());assert err<1e-6,(c,key,err)
    ground[key]=dict(unit=unit,min=float(yy.min()),max=float(yy.max()),expected=value,max_abs_error=err)
   yy=v[:,aliases['Y_'+wheel][0]].astype(float);lat=yy+16;assert abs(lat).max()<2000
   boundary[wheel]=dict(wheel_center_L_min_m=float(lat.min()),wheel_center_L_max_m=float(lat.max()),wheel_center_boundary_margin_m=float(2000-abs(lat).max()))
  rows.append(dict(case=c,actual_end_s=m['actual_end_s'],classification=m['classification'],raw_sha256=idx['archive_sha256'],common_bitwise_equal=reg['all_common_bitwise_equal'],echo_surface_verified=keys,actual_ground_outputs=ground,wheel_center_boundary_diagnostic=boundary,ground_evidence_step_s=float(tv[1]-tv[0]),warning_lines=[s for s in log.splitlines() if 'extrapolated' in s]))
 dump(W/'audit/final_road_validation.json',dict(passed=True,frozen_execution_files=len(freeze['files']),protected_files=len(freeze['external_files']),cases=rows,old_failures_preserved=True,GUI_not_changed=True))
 print('FINAL_ROAD_VALIDATION_PASS',[(x['case'],x['classification']) for x in rows])
if __name__=='__main__':main()
