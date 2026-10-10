"""只读冻结检查、指标独立复算及派生缓存重建证明；不启动车辆。"""
from pathlib import Path
import hashlib,json,subprocess,sys
W=Path(__file__).resolve().parents[1]
def read(p):return json.loads(p.read_text(encoding='utf-8-sig'))
def sha(p):
 h=hashlib.sha256()
 with p.open('rb') as f:
  for b in iter(lambda:f.read(8*1024*1024),b''):h.update(b)
 return h.hexdigest()
def snap(root):return {p.relative_to(W).as_posix():dict(sha256=sha(p),bytes=p.stat().st_size) for p in sorted(root.rglob('*')) if p.is_file()}
freeze=read(W/'source/runner_freeze.json')
for key in ['files','external_files']:
 for x in freeze[key]:
  p=Path(x['path']) if key=='external_files' else W/x['path'];assert sha(p)==x['sha256'],str(p)
cases=read(W/'source/protocol.json')['cases'];raw=[];reviews={}
for case in cases:
 file=W/f'audit/review_{case}.json';reviews[case]=file.read_bytes();r=read(file);p=W/f'results/runs/LeftRight_R3_{case}/raw_numeric.mat'
 assert sha(p)==r['raw_sha256'];raw.append(dict(path=p.relative_to(W).as_posix(),sha256=sha(p),bytes=p.stat().st_size))
 result=subprocess.run([sys.executable,'-X','utf8',str(W/'internal/review_case.py'),case],capture_output=True,text=True,encoding='utf-8')
 assert result.returncode==0,result.stderr
 assert file.read_bytes()==reviews[case],'独立复算结果变化'
before=snap(W/'cache');metric_before={k:v for k,v in snap(W/'results/analysis').items() if not k.endswith('report.md')}
result=subprocess.run([sys.executable,'-X','utf8',str(W/'internal/compare_cases.py')],capture_output=True,text=True,encoding='utf-8');assert result.returncode==0,result.stderr
assert before==snap(W/'cache'),'缓存重建不一致'
assert metric_before=={k:v for k,v in snap(W/'results/analysis').items() if not k.endswith('report.md')},'指标重建不一致'
out=dict(passed=True,frozen_sources=len(freeze['files']),protected_external=len(freeze['external_files']),vehicle_runs=5,completed_60s=sum(read(W/f'audit/review_{case}.json')['actual_end_s']>=60 for case in cases),raw=raw,raw_bytes=sum(x['bytes'] for x in raw),cache=before,cache_bytes=sum(x['bytes'] for x in before.values()),cache_bitwise_rebuilt=True,metrics_exact_rebuilt=True)
(W/'audit/final_integrity.json').write_text(json.dumps(out,ensure_ascii=False,indent=2),encoding='utf-8');print(json.dumps({k:v for k,v in out.items() if k not in ['raw','cache']},ensure_ascii=False))
