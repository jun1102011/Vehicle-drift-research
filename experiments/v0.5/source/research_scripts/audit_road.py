"""只读核查原道路错误、圆坐标条件和新旧配置字节差异；不启动车辆。"""
from prepare_research import *
import math,difflib
from prepare_state_configs import build_case,straighten
def main():
 echo=P/'results/runs/LeftRight_R3_kf384_kr384/LastRun_echo.par.evidence'
 text=echo.read_text(encoding='utf-8',errors='replace')
 expected={'NROAD_DZ':'0','ROAD_ZS_CONSTANT(1)':'0','MU_ROAD_CONSTANT(1)':'0.85','MU_ROAD_L_CONSTANT(1)':'0','RR_SURF(1)':'1','OPT_DM':'0','OPT_INIT_PATH':'0','ROAD_SMAX_NEW_ID(1)':'0','ROAD_SMIN_NEW_ID(1)':'0'}
 for k,v in expected.items():
  h=re.findall(r'^'+re.escape(k)+r'\s+(\S+)',text,re.M);assert h==[v],(k,h)
 cases=[];baseline=None
 for c in CASES:
  cfg,par,sim=build_case(c);old=Path(cfg['origin_source_path']).read_bytes()
  assert par==straighten(old)
  dif=list(difflib.unified_diff(old.decode('utf-8',errors='replace').splitlines(),par.decode('utf-8',errors='replace').splitlines(),fromfile='old_circle',tofile='straight_coordinates'))
  normalized=re.sub(rb'(?m)^MX_AUX_COEFFICIENT[^\r\n]*',b'MX_AUX_COEFFICIENT <case>',par)
  if baseline is None:baseline=normalized
  else:assert normalized==baseline,'不同组存在非Aux差异'
  log=P/f'results/runs/LeftRight_R3_{c}/LastRun_log.txt.evidence';s=log.read_text(encoding='utf-8',errors='replace')
  hit=re.search(r'T = ([\d.]+) because\s+S and L values.*?Best estimate: S = ([-\d.eE]+) and L = ([-\d.eE]+) when X = ([-\d.eE]+) and Y = ([-\d.eE]+)',s,re.S)
  info=dict(case=c,old_log=str(log),old_log_sha256=sha(log),old_par_sha256=sha(Path(cfg['origin_source_path'])),new_par_sha256=hashlib.sha256(par).hexdigest(),diff=dif)
  if hit:
   t,S,L,X,Y=map(float,hit.groups());dist=math.hypot(X,Y+1)
   info['error']=dict(t_s=t,best_S=S,best_L=L,query_X=X,query_Y=Y,center_distance_m=dist,jacobian_at_best_L=1-L/15,geometric_inverse_sensitivity_R_over_distance=15/dist,query_point_identity='not exposed in error; do not assume CG or particular wheel')
  cases.append(info)
 dump(W/'audit/road_diagnosis.json',dict(verified=True,physical_surface_from_echo=expected,echo_source=str(echo),echo_sha256=sha(echo),manual=str(Path(r'C:\Program Files (x86)\CarSim2020.0_Prog\Help\Screens\roads.pdf')),manual_sha256=sha(Path(r'C:\Program Files (x86)\CarSim2020.0_Prog\Help\Screens\roads.pdf')),manual_pages=[3,4,5,6],circle=dict(start=[0,-16],heading_deg=0,radius_m=15,center=[0,-1],transform=['X=(15-L)*sin(S/15)','Y=-1-(15-L)*cos(S/15)'],jacobian='1-L/15',interpretation='circle center singular; observed errors nearby, iterative algorithm itself not instrumented'),straight=dict(start=[0,-16],heading_deg=0,length_m=2000,loop=False,transform=['S=X','L=Y+16'],jacobian=1),cases=cases,only_aux_differences_between_cases=True))
 print('ROAD_AUDIT_PASS',[(x['case'],x.get('error',{}).get('center_distance_m')) for x in cases])
if __name__=='__main__':main()
