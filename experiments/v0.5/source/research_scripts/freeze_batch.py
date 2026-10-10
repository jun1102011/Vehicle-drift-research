"""编译后冻结五组执行文件；只读父版本与物理配置身份。"""
from prepare_research import *
assert read(W/'audit/compile_validation.json')['passed']
assert read(W/'audit/logic_validation.json')['passed']
ext=read(W/'audit/source_before.json')
for p,h in ext.items():assert sha(Path(p))==h,p
for x in read(W/'audit/preparation.json')['identical_control_files']:assert sha(W/x['path'])==x['sha256']
execution={'state_cycle_core.m','native_state_capture.m','run_state_only_case.m','prepare_state_configs.py','validate_compile.m','test_logic.m'}
files=[dict(path=p.relative_to(W).as_posix(),sha256=sha(p)) for d in ['model','source','internal'] for p in sorted((W/d).rglob('*')) if p.is_file() and 'slprj' not in p.parts and '__pycache__' not in p.parts and p.suffix in ['.py','.m','.slx','.json','.txt','.evidence'] and p.name!='runner_freeze.json' and (d!='internal' or p.name in execution)]
assert not (W/'source/runner_freeze.json').exists()
dump(W/'source/runner_freeze.json',dict(files=files,external_files=[dict(path=p,sha256=h) for p,h in ext.items()],preparation_reviewed=True,compiled_and_reviewed=True))
print('FROZEN',len(files),len(ext))
