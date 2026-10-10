"""派生道路坐标修复副本；不启动车辆。保持控制字节与历史证据。

仅将原圆形道路参考坐标换成直线：保持Y=-16、朝向0，S=X，L=Y+16。
物理路面Z=0、mu=.85、RR=1和无驾驶员/无路径初值均由原Echo验证。
"""
from pathlib import Path
import hashlib,json,shutil,re
W=Path(__file__).resolve().parents[1];P=W.parent/'step37b_bidirectional_stiffness_20261010'
CASES=['kf384_kr384','kf192_kr384','kf576_kr384','kf384_kr192','kf384_kr576']
def read(p):return json.loads(p.read_text(encoding='utf-8-sig'))
def sha(p):return hashlib.sha256(p.read_bytes()).hexdigest()
def dump(p,x):p.parent.mkdir(parents=True,exist_ok=True);p.write_text(json.dumps(x,ensure_ascii=False,indent=2),encoding='utf-8')
def main():
 assert not (W/'source/protocol.json').exists(),'拒绝覆盖已准备实验'
 for d in ['results/runs','results/metrics','results/analysis','source/profiles','audit','internal']:(W/d).mkdir(parents=True,exist_ok=True)
 ext={x['path']:x['sha256'] for x in read(P/'source/runner_freeze.json')['external_files']}
 ext.update({str(P/x['path']):x['sha256'] for x in read(P/'MANIFEST.json')['files']})
 for p,h in ext.items():assert sha(Path(p))==h,p
 dump(W/'audit/source_before.json',ext)
 shutil.copytree(P/'model',W/'model',ignore=shutil.ignore_patterns('slprj','*.slxc','__pycache__'))
 for p in (P/'source/profiles').glob('*.json'):shutil.copy2(p,W/'source/profiles'/p.name)
 shutil.copy2(P/'source/public_executor_frozen.txt',W/'source/public_executor_frozen.txt')
 for name in ['state_cycle_core.m','native_state_capture.m','validate_compile.m','test_logic.m','compare_cases.py','render_comparison.m']:
  shutil.copy2(P/'internal'/name,W/'internal'/name)
 cfg=(P/'internal/prepare_state_configs.py').read_text(encoding='utf-8')
 cfg=cfg.replace('step37b_state_only_20261010','step37c_road_coordinate_20261010').replace('step37b-state-only-config','step37c-road-coordinate-config')
 pos=cfg.index('def build_case(')
 cfg=cfg[:pos]+'''def straighten(par: bytes) -> bytes:
    # 唯一白名单：闭合开关、段类型、半径/圆心角替换为直线长度；其他字节保持。
    assert par.count(b"OPT_PATH_LOOP 1") == 1
    assert par.count(b"SEGMENT_TYPE 3") == 1
    par = par.replace(b"OPT_PATH_LOOP 1", b"OPT_PATH_LOOP 0")
    par = par.replace(b"SEGMENT_TYPE 3", b"SEGMENT_TYPE 0")
    pattern = rb"SEGMENT_RADIUS 15\\.000000(\\r?\\n)SEGMENT_ARC 360\\.000000"
    assert len(re.findall(pattern,par)) == 1
    return re.sub(pattern,b"SEGMENT_LENGTH 2000",par)


'''+cfg[pos:]
 needle='    audit_par(par)\n    audit_sim(sim)'
 assert cfg.count(needle)==1
 cfg=cfg.replace(needle,'    audit_par(par)\n    par = straighten(par)\n    audit_sim(sim)')
 cfg=cfg.replace('file_derived_from_step35b_approved_60s','file_derived_road_coordinates_straight_uniform_plane')
 cfg=cfg.replace('"origin_source_path": str(old_par_path), "origin_source_sha256": digest(par)','"origin_source_path": str(old_par_path), "origin_source_sha256": file_digest(old_par_path)')
 cfg=cfg.replace('par 没有任何修改。','par 仅作白名单道路坐标修改。')
 cfg=cfg.replace('Run_all.par: no bytes changed','Run_all.par: only OPT_PATH_LOOP 1->0, SEGMENT_TYPE 3->0, radius/arc -> SEGMENT_LENGTH 2000; physical flat road unchanged')
 (W/'internal/prepare_state_configs.py').write_text(cfg,encoding='utf-8')
 runner=(P/'internal/run_state_only_case.m').read_text(encoding='utf-8')
 runner=runner.replace('step37b_state_only_20261010','step37c_road_coordinate_20261010').replace('step37b-state-only-config','step37c-road-coordinate-config').replace('STEP37B_ARCHIVED','STEP37C_ARCHIVED')
 runner=runner.replace('file_derived_from_step35b_approved_60s','file_derived_road_coordinates_straight_uniform_plane')
 needle="assert(strcmp(actual,base),'Run_all.par物理参数不允许任何字节变化');"
 assert needle in runner
 runner=runner.replace(needle,"""% 本轮唯一变化是道路坐标；不改物理高度/附着/车辆/初态。
expected=replace_line(base,'OPT_PATH_LOOP','OPT_PATH_LOOP 0');
expected=replace_line(expected,'SEGMENT_TYPE','SEGMENT_TYPE 0');
expected=regexprep(expected,'SEGMENT_RADIUS 15\\.000000\\r?\\nSEGMENT_ARC 360\\.000000','SEGMENT_LENGTH 2000');
assert(strcmp(actual,expected),'道路坐标白名单以外字节变化');""")
 runner=runner.replace('Step37b frozen left-right controller; stiffness only; no ASB','Step37c uniform-plane straight road coordinates; frozen left-right controller; no ASB')
 (W/'internal/run_state_only_case.m').write_text(runner,encoding='utf-8')
 review=(P/'internal/review_case.py').read_text(encoding='utf-8')
 st=review.index(" if case=='kf384_kr384':");en=review.index(' result=dict(',st)
 review=review[:st]+''' # 每组与同刚度旧记录核对共同有效前缀，不裁掉旧末端异常样本。
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
'''+review[en:]
 (W/'internal/review_case.py').write_text(review,encoding='utf-8')
 freeze=(P/'internal/freeze_batch.py').read_text(encoding='utf-8').replace('from prepare_batch import *','from prepare_research import *')
 (W/'internal/freeze_batch.py').write_text(freeze,encoding='utf-8')
 same=[]
 for rel in ['model/stabilizer_bar_step37a.slx','model/a001_step37a_initialize.m','internal/state_cycle_core.m','internal/native_state_capture.m','source/profiles/LeftRight_R3.json']:
  assert sha(W/rel)==sha(P/rel);same.append(dict(path=rel,sha256=sha(W/rel)))
 dump(W/'audit/preparation.json',dict(passed=True,identical_control_files=same,protected_count=len(ext),vehicle_started=False))
 dump(W/'source/protocol.json',dict(name='Step37c 道路坐标异常隔离与五刚度补齐',cases=CASES,max_full_runs=5,duration_s=60,controller='Step37b exact bytes',road='non-loop straight reference, same homogeneous flat plane',whitelist=['OPT_PATH_LOOP 1->0','SEGMENT_TYPE 3->0','SEGMENT_RADIUS/ARC -> SEGMENT_LENGTH 2000'],baseline_gate='56 common numeric channels bitwise identical full60s',other_gate='56 common available prefix identical; any unexplained difference stops',criteria='unchanged 600/20000/2000 native samples',no_ASB=True,no_path=True,expected_raw_and_evidence_GB=.45,GUI_changed=False))
 print('PREPARED',len(ext))
if __name__=='__main__':main()
