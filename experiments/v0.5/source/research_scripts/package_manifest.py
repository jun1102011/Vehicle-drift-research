"""清理后的全批身份与交付清单；不重新运行或修改原始记录。"""
from prepare_research import *
f=read(W/'source/runner_freeze.json')
for key in ['files','external_files']:
 for x in f[key]:assert sha(Path(x['path']) if key=='external_files' else W/x['path'])==x['sha256'],x['path']
a=read(W/'audit/final_integrity.json')
for x in a['raw']:assert sha(W/x['path'])==x['sha256']
assert not (W/'cache').exists() and not (W/'model/slprj').exists()
a.pop('full_runs',None);a['vehicle_runs']=5;a['completed_60s']=sum(read(W/f'audit/review_{c}.json')['actual_end_s']>=60 for c in CASES);a['interrupted_runs']=5-a['completed_60s'];a['post_cleanup_integrity_pass']=True;dump(W/'audit/final_integrity.json',a)
files=[dict(path=p.relative_to(W).as_posix(),bytes=p.stat().st_size,sha256=sha(p)) for p in sorted(W.rglob('*')) if p.is_file() and p.name!='MANIFEST.json' and '__pycache__' not in p.parts]
dump(W/'MANIFEST.json',dict(post_cleanup_integrity_pass=True,vehicle_runs=5,completed_60s=a['completed_60s'],files=files));print('PACKAGE_PASS',len(files),sum(x['bytes'] for x in files))
