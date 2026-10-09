"""原生数据独立复算新总表关键项；不运行车辆，不改历史。"""
import sys,csv,json
from pathlib import Path
import numpy as np
sys.dont_write_bytecode=True
from review_case import W,raw,read,dump,sha
rows=list(csv.DictReader((W/'results/analysis/summary.csv').open(encoding='utf-8-sig')))
checks=[];base_ref=None
def check(case,key,a,b,atol=1e-10):
 assert np.allclose(a,b,atol=atol,rtol=1e-11),(case,key,a,b)
 checks.append({'case':case,'metric':key,'max_abs_difference':float(np.max(abs(np.asarray(a)-np.asarray(b))))})
for row in rows:
 cid=row['case'];run=W/'results/runs'/('V12_beta30_state_only_R3_S_'+cid)
 p=raw.load(run,'repair_plant_raw');e=raw.load(run,'repair_entry_diag');a=raw.load(run,'repair_actuator_command')
 start=e[np.flatnonzero(e[:,1]==1)[0],0];i0=int(round(start/.0005));sl=slice(i0,i0+16000);ps=slice(i0+3000,i0+4000)
 assert len(p[sl])==16000 and len(p[ps])==1000
 x=np.column_stack((np.sqrt(p[:,2]**2+p[:,3]**2)/3.6,p[:,4],p[:,5]*np.pi/180))
 # 独立按时间戳查找零阶保持参考，避免沿用原脚本的整除索引。
 ix=np.searchsorted(e[:,0],np.round(p[:,0],7)+1e-10,side='right')-1
 ref=e[ix,16:19].copy();ref[:,1]*=180/np.pi
 if base_ref is None:base_ref=ref
 check(cid,'shared_reference',ref[sl],base_ref[sl])
 for k,name in enumerate(['V','beta','AVz']):
  for win,label in [(sl,'8'),(ps,'_pulse_')]:
   er=x[win,k]-base_ref[win,k];val=np.dot(er,er)*.0005
   key=('Jshared8_'+name)if label=='8'else('Jshared_pulse_'+name)
   check(cid,key,val,float(row[key]),1e-8)
 ok=(np.abs(x-[12,-30,.5989797412715093])<=[.2,2,.03]).all(axis=1)&(x[:,2]>0)&(p[:,0]>=start)
 # 使用区段边界直接找长度，而非原指标的逐步计数或累计和。
 edges=np.diff(np.r_[False,ok,False].astype(int));left=np.flatnonzero(edges==1);right=np.flatnonzero(edges==-1)
 first=left[np.flatnonzero(right-left>=600)[0]]
 check(cid,'entry_s',p[first,0]-start,float(row['entry_s']))
 check(cid,'entry_confirm_s',p[first+599,0]-start,float(row['entry_confirm_s']))
 check(cid,'V_min_0_8',x[sl,0].min(),float(row['V_min_0_8']))
 check(cid,'Fz_min_0_8',p[sl,17:21].min(),float(row['Fz_min_0_8']))
 v=a[sl,5:9].ravel();check(cid,'torque_RMS_0_8',np.linalg.norm(v)/np.sqrt(len(v)),float(row['torque_RMS_0_8']),1e-8)
 check(cid,'torque_peak_0_8',np.abs(v).max(),float(row['torque_peak_0_8']))
 assert sha(run/'raw_numeric.mat')==read(run/'archive_index.json')['archive_sha256']
dump(W/'audit/independent_summary_verification.json',{'passed':True,'check_count':len(checks),'checks':checks,'new_vehicle_runs':0})
print('INDEPENDENT_SUMMARY_PASS',len(checks))

