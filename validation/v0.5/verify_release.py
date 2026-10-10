from pathlib import Path
import hashlib,json,subprocess,re
from urllib.parse import unquote
R=Path('E:/00硕士毕业论文/GitHub发布/stabilizer-bar-drift')
W=Path('E:/00硕士毕业论文/第四章_赵选铭漂移复现_维修版/分步维修记录/step37c_road_coordinate_20261010')
A=Path(__file__).parent; E=R/'experiments/v0.5'
def sha(p):return hashlib.sha256(p.read_bytes()).hexdigest()
def read(p):return json.loads(p.read_text(encoding='utf-8-sig'))
def dump(p,x):p.parent.mkdir(parents=True,exist_ok=True);p.write_text(json.dumps(x,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
def git(*args):return subprocess.check_output(['git',*args],cwd=R)
for x in read(W/'MANIFEST.json')['files']:assert sha(W/x['path'])==x['sha256'],x['path']
mapping=read(E/'source/publication_sources.json')['mapping']
for x in mapping:assert sha(Path(x['source']))==sha(R/x['destination'])==x['sha256'],x['destination']
for x in read(E/'evidence/raw_data_index.json')['records']:assert sha(Path(x['path']))==x['sha256']
assert read(E/'evidence/publication_validation.json')['passed']
assert git('rev-parse','V0.4^{}').decode().strip()=='6de15ed2d5adf77b1b3e1ea6f870f78ed4143cbe'
allowed={'.gitignore','README.md','CHANGELOG.md','MANIFEST.sha256'}
changed=git('diff','--name-only','-z').decode('utf-8').split('\0')
assert set(filter(None,changed))<=allowed,changed
# 对未改的既有文件按Git对象逐字节核对，包含中文路径。
old_count=0
for path in filter(None,git('ls-tree','-r','--name-only','-z','HEAD').decode('utf-8').split('\0')):
 if path in allowed:continue
 assert (R/path).read_bytes()==git('show','HEAD:'+path),path
 old_count+=1
rows=read(R/'docs/v0.5/summary.json');assert len(rows)==5 and all(x['actual_end_s']==60 for x in rows)
issues=[]; checked=0
docs=[R/'README.md',R/'docs/v0.5/report.md',E/'README.md']
for doc in docs:
 for target in re.findall(r'\]\(([^)]+)\)',doc.read_text(encoding='utf-8')):
  if target.startswith(('http:','https:','#')):continue
  p=unquote(target.split('#')[0]).strip('<>');checked+=1
  if not (doc.parent/p).exists():issues.append((str(doc),p))
assert not issues,issues
files=list((R/'docs/v0.5').rglob('*'))+list(E.rglob('*'))
published=[p for p in files if p.is_file() and not any(x in p.parts for x in ['slprj','__pycache__']) and p.suffix!='.slxc']
assert not any(p.name=='raw_numeric.mat' or '.vsb' in p.name for p in published)
assert max(p.stat().st_size for p in published)<10*1024*1024
dump(E/'evidence/publication_integrity.json',dict(passed=True,copied_sources_verified=len(mapping),history_unchanged_files=old_count,local_links_checked=checked,raw_hashes_verified=5,source_manifest_verified=True,original_V04_unchanged=True,new_vehicle_simulations=0,big_raw_uploaded=False))
# 全仓清单涵盖已跟踪与本次新发布文件，排除自身和Git忽略缓存。
names=sorted(set(filter(None,git('ls-files','--cached','--others','--exclude-standard','-z').decode('utf-8').split('\0')))-{'MANIFEST.sha256'})
assert all(not n.endswith('.vsb') and not n.endswith('raw_numeric.mat') for n in names)
(R/'MANIFEST.sha256').write_text(''.join(f'{sha(R/n)}  {n}\n' for n in names),encoding='utf-8',newline='\n')
print(json.dumps(dict(passed=True,old_files_unchanged=old_count,copied_sources=len(mapping),manifest_files=len(names),v05_files=len(published),v05_bytes=sum(p.stat().st_size for p in published)),ensure_ascii=False))
