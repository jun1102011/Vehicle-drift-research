function [command,diag,cycle_diag]=cycle_control(y,t,entry_cfg,repair_cfg,cycle_cfg,stadium_packet)
%#codegen
% 实际控制链：工作点/局部状态反馈→前轮角与轴滑移→公共轮速和转向执行。
% y为CarSim原50路采样输出，轮序FL/FR/RL/RR；command=[delta(rad);T四轮(Nm)]。
% entry_cfg冻结已验证起漂库；cycle_cfg含退出库、长度、维持时间和接管门槛。
% 原路径LQR与式4.10关闭。此函数不恢复车辆状态、不调用轮胎Fiala反演执行输入。
% diag保持原70项顺序；cycle_diag另记退出阶段、门控、进度与参考导数。
persistent phase distance elapsed readyTime captureTime satTime mix mixStart mixTarget mixElapsed prepareFailed vprev wprev initialized protection
persistent holdTime exitDistance straightCapture straightMix errors sats historyCount previousRef refInitialized
if isempty(initialized)
 phase=0;distance=0;elapsed=0;readyTime=0;captureTime=0;satTime=0;mix=0;mixStart=0;mixTarget=0;mixElapsed=0;prepareFailed=false;
 vprev=zeros(3,1);wprev=zeros(4,1);initialized=false;protection=false;
 holdTime=0;exitDistance=0;straightCapture=0;straightMix=0;errors=zeros(500,3);sats=false(500,1);historyCount=0;previousRef=zeros(7,1);refInitialized=false;
end
command=zeros(5,1);diag=zeros(70,1);cycle_diag=zeros(16,1);
if repair_cfg.entry_mode==0,return;end
Ts=repair_cfg.Ts;Rw=repair_cfg.Rw;Iw=repair_cfg.Iw;
x=[hypot(y(2),y(3))/3.6;y(4)*pi/180;y(5)*pi/180;mean(y(27:28));mean(y(29:30));y(8)*pi/180;y(26)*pi/180];
omega=y(21:24)*pi/30;Fz=y(17:20);last=entry_cfg.nodes(7);oldphase=phase;
assert(all(isfinite(x))&&all(isfinite(Fz)),'Nonfinite state or wheel load');first=~initialized;
if first,vprev=entry_cfg.normal.v;wprev=omega;initialized=true;end
elapsed=elapsed+Ts;if protection,satTime=satTime+Ts;else,satTime=0;end
errors(1:499,:)=errors(2:500,:);errors(500,:)=(x(1:3)-last.x(1:3))';sats(1:499)=sats(2:500);sats(500)=protection;historyCount=min(500,historyCount+1);
inside=all(abs(x-last.x)<=entry_cfg.thresholds)&&all(Fz>0)&&satTime<.2;
exitProgress=0;substage=0;holdgate=false;progress=smooth5(distance/entry_cfg.length);weight=0;ref=entry_cfg.normal;request=zeros(3,1);

% 操场模式由路径层提供唯一参考与反馈请求；共用下面唯一执行器历史。
if repair_cfg.stadium_mode==0
% 起漂段完全保留v0.1状态机和插值，新增退出仅在局部维持验收之后启用。
if phase<4
 if phase==0
  ready=abs(x(1)-12)<=.2&&abs(x(2))<=pi/180&&abs(x(3))<=.02;
  if ready,readyTime=readyTime+Ts;else,readyTime=0;end
  if readyTime>=1-1e-9&&~prepareFailed,phase=1;elseif elapsed>=10,prepareFailed=true;end
 else
  if phase==1,distance=distance+max(x(1),0)*Ts;end
  if inside,captureTime=captureTime+Ts;else,captureTime=0;end
  if phase==1&&captureTime>=.3-1e-9,phase=2;mixStart=mix;mixTarget=1;mixElapsed=0;end
  if (phase==2||phase==3)&&~inside&&mixTarget~=0,phase=2;mixStart=mix;mixTarget=0;mixElapsed=0;end
  if phase==2
   mixElapsed=mixElapsed+Ts;mix=mixStart+(mixTarget-mixStart)*smooth5(mixElapsed/.5);
   if mixElapsed>=.5-1e-9,if mixTarget==1,phase=3;else,phase=1;end,end
  end
 end
 progress=smooth5(distance/entry_cfg.length);
 if phase==0,ref=mixnode(entry_cfg.normal,entry_cfg.nodes(1),smooth5(t/2));
 else,ref=sampleNodes(entry_cfg.nodes,entry_cfg.grid,progress);end
 request=(1-mix)*(ref.v-feedback_delta(ref.K,x-ref.x))+mix*(last.v-feedback_delta(last.K,x-last.x));ref=mixnode(ref,last,mix);
 weight=mix;
 if phase==3&&oldphase==3,holdTime=holdTime+Ts;else,holdTime=0;end
 % drift_cycle保持末5秒门控；研究drift_turn按完整稳定弧验收，切换只检末0.5秒。
 historyRequired=500;if cycle_cfg.scheme==3,historyRequired=50;end
 if repair_cfg.cycle_mode==1&&holdTime>=cycle_cfg.hold_min_s-1e-8&&historyCount>=historyRequired
  tol=cycle_cfg.drift_tolerance;
  if cycle_cfg.scheme==3
   rmse=sqrt(mean(errors(451:500,:).^2,1))';trend=mean(errors(491:500,:),1)'-mean(errors(451:460,:),1)';
   satFraction=mean(double(sats(451:500)));firstHistory=451;
  else
   rmse=sqrt(mean(errors.^2,1))';trend=mean(errors(401:500,:),1)'-mean(errors(1:100,:),1)';
   satFraction=mean(double(sats));firstHistory=1;
  end
  run=0;longest=0;for k=firstHistory:500,if sats(k),run=run+1;else,run=0;end;longest=max(longest,run);end
  holdgate=all(rmse<=tol)&&all(abs(trend)<=tol)&&satFraction<=.05&&longest*Ts<=.2&&all(Fz>0);
  if holdgate,phase=4;exitDistance=0;end
 end
end

% 退出库独立。原B保持两段退出；scheme=3连接漂移点→r=.3普通弯→直行，滑移卸载与横摆下降重叠。
% 直行接管使用同一执行器历史，不复位车辆，也不叠加两套轮速环输出。
if phase>=4
 if phase==4&&oldphase>=4,exitDistance=exitDistance+max(x(1),0)*Ts;end
 q=min(exitDistance/cycle_cfg.length,1);
 if cycle_cfg.scheme==2
  if q<.5,exitProgress=.5*smooth5(2*q);else,exitProgress=.5+.5*smooth5(2*q-1);end
 else,exitProgress=smooth5(q);end
 if exitProgress<.5,substage=1;else,substage=2;end
 ref=sampleNodes(cycle_cfg.nodes,cycle_cfg.grid,exitProgress);normal=cycle_cfg.nodes(end);
 inside=all(abs(x-normal.x)<=cycle_cfg.straight_capture)&&all(Fz>0)&&~protection&&exitProgress>=1-1e-8;
 if inside,straightCapture=straightCapture+Ts;else,straightCapture=0;end
 if phase==4&&straightCapture>=.3-1e-8,phase=5;straightMix=0;end
 if phase==5
  straightMix=min(1,straightMix+Ts/.5);
  if ~inside,phase=4;elseif straightMix>=1-1e-8,phase=6;end
 end
 weight=0;if phase==5,weight=smooth5(straightMix);elseif phase==6,weight=1;end
 request=(1-weight)*(ref.v-feedback_delta(ref.K,x-ref.x))+weight*(normal.v-feedback_delta(normal.K,x-normal.x));ref=mixnode(ref,normal,weight);
end
else
 ref.x=stadium_packet(1:7);ref.v=stadium_packet(8:10);ref.offset=stadium_packet(11:14);ref.K=zeros(3,5);
 request=stadium_packet(15:17);phase=stadium_packet(18);progress=stadium_packet(19);mix=1;weight=1;
end
rates=zeros(7,1);if refInitialized,rates=(ref.x-previousRef)/Ts;end;previousRef=ref.x;refInitialized=true;
cycle_diag=[phase;exitProgress;substage;holdTime;double(holdgate);double(inside);straightCapture;weight;exitDistance;rates];

% 公共执行层：真实接地点速度、实际Fx负载补偿与冻结的局部执行预算。
lo=[ref.v(1)-entry_cfg.steer_deviation;entry_cfg.slip_low];hi=[ref.v(1)+entry_cfg.steer_deviation;entry_cfg.slip_high];
v=min(max(request,lo),hi);slew=[entry_cfg.steering_rate_rad_s;entry_cfg.slip_rate;entry_cfg.slip_rate];v=min(max(v,vprev-slew*Ts),vprev+slew*Ts);vprev=v;
contact=Rw*omega./(1+y(27:30));assert(all(isfinite(contact))&&all(contact>0),'Invalid contact velocity');
candidate=contact/Rw.*(1+[v(2);v(2);v(3);v(3)]);loadTorque=Rw*y(9:12)+ref.offset;kw=entry_cfg.wheel_kw;
if first,wd=omega;dwd=zeros(4,1);
else
 gain=kw+Iw/Ts;low=(-entry_cfg.torque_limit+kw*omega+Iw/Ts*wprev-loadTorque)/gain;high=(entry_cfg.torque_limit+kw*omega+Iw/Ts*wprev-loadTorque)/gain;
 low=max(low,wprev-entry_cfg.omega_acceleration*Ts);high=min(high,wprev+entry_cfg.omega_acceleration*Ts);assert(all(low<=high),'Empty torque and wheel acceleration budget');
 wd=min(max(candidate,low),high);dwd=(wd-wprev)/Ts;
end
feedback=kw*(wd-omega);inertia=Iw*dwd;raw=feedback+inertia+loadTorque;T=min(max(raw,-entry_cfg.torque_limit),entry_cfg.torque_limit);wprev=wd;
protection=any(abs(T)>=entry_cfg.torque_limit-1e-6);command=[v(1);T];
diag=[phase;progress;mix;double(inside);double(prepareFailed);readyTime;captureTime;satTime;x;ref.x;ref.v;request;v;candidate;wd;omega;feedback;inertia;loadTorque;raw;T;double(any(abs(v-request)>1e-9));double(any(abs(wd-candidate)>1e-9));double(protection);contact];
end
function h=smooth5(q)
q=min(max(q,0),1);h=10*q^3-15*q^4+6*q^5;
end
function out=mixnode(a,b,f)
out=a;out.x=(1-f)*a.x+f*b.x;out.v=(1-f)*a.v+f*b.v;out.K=(1-f)*a.K+f*b.K;out.offset=(1-f)*a.offset+f*b.offset;
end
function out=sampleNodes(nodes,grid,s)
j=1;for k=1:numel(grid)-1,if s>=grid(k),j=k;end,end
f=(s-grid(j))/(grid(j+1)-grid(j));out=mixnode(nodes(j),nodes(j+1),f);
end


function du=feedback_delta(K,error)
% 五项误差反馈：车辆侧倾仍存在，但不直接进入转角/滑移请求。
% K为3×5，状态前五项顺序为V、beta、AVz、前后轴滑移。
du=K*error(1:5);
end
