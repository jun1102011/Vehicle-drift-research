function [command,diag,cycle_diag,schedule_diag]=cycle_control(y,t,entry_cfg,repair_cfg,cycle_cfg,stadium_packet,state_cfg)
%#codegen
% 无路径任务：时间参考→五状态反馈→原公共执行层。这里不读取XY/曲率/方向。
% y是CarSim原50路，轮序FL/FR/RL/RR；stadium_packet现只输送原生600点事件。
% Roll/AVx仅用于原物理接管，K只乘[V,beta,AVz,kappa_f,kappa_r]。
% 进入固定10s维持后不回退/重计；误差与退出门控分开，失败也保留退漂证据。
persistent phase readyTime elapsed entryStart captureTime satTime mixStart holdStart exitStart straightCapture straightMix previousRef refInitialized vprev wprev initialized protection turnStart rightCapture rightMix rightHold rightExit rightStraightCapture rightStraightMix
if isempty(initialized)
 phase=0;readyTime=0;elapsed=0;entryStart=0;captureTime=0;satTime=0;mixStart=0;holdStart=0;exitStart=0;straightCapture=0;straightMix=0;
 turnStart=0;rightCapture=0;rightMix=0;rightHold=0;rightExit=0;rightStraightCapture=0;rightStraightMix=0;
 previousRef=zeros(7,1);refInitialized=false;vprev=zeros(3,1);wprev=zeros(4,1);initialized=false;protection=false;
end
command=zeros(5,1);diag=zeros(70,1);cycle_diag=zeros(16,1);schedule_diag=zeros(25,1);gainProgress=0;nodeProgress=0;nodeIndex=1;nodeFraction=0;
Ts=repair_cfg.Ts;Rw=repair_cfg.Rw;Iw=repair_cfg.Iw;
x=[hypot(y(2),y(3))/3.6;y(4)*pi/180;y(5)*pi/180;mean(y(27:28));mean(y(29:30));y(8)*pi/180;y(26)*pi/180];
omega=y(21:24)*pi/30;Fz=y(17:20);last=entry_cfg.nodes(end);
assert(all(isfinite(x))&&all(isfinite(Fz)),'Nonfinite state or wheel load');first=~initialized;
if first,vprev=entry_cfg.normal.v;wprev=omega;initialized=true;end
elapsed=elapsed+Ts;if protection,satTime=satTime+Ts;else,satTime=0;end
inside=all(abs(x-last.x)<=entry_cfg.thresholds)&&all(Fz>0)&&satTime<.2;
progress=0;mix=0;weight=0;holdTime=0;exitProgress=0;substage=0;exitElapsed=0;holdgate=false;prepareFailed=false;
ref=entry_cfg.normal;
% 准备仍由实际直行状态持续1s确定，不使用道路站点或世界坐标。
if phase==0
 ready=abs(x(1)-12)<=.2&&abs(x(2))<=pi/180&&abs(x(3))<=.02;
 if ready,readyTime=readyTime+Ts;else,readyTime=0;end
 ref=mixnode(entry_cfg.normal,entry_cfg.nodes(1),smooth5(t/2));
 if readyTime>=1-1e-9&&elapsed<10,phase=1;entryStart=t;elseif elapsed>=10,prepareFailed=true;end
end
if phase>=1&&phase<=3
 q=max(0,t-entryStart);progress=min(q/state_cfg.entry_reference_s,1);
 % 连续经过节点：预加载复用原公式；其后β一次平滑到−30°。
 % β同时作为原银行的调度坐标，x/v/K/offset共享一个插值比例。
 if q<state_cfg.preload_end_s
  ref=sampleNodes(entry_cfg.nodes(1:7),entry_cfg.grid,smooth5(q/state_cfg.prefix_s));
 else
  h=smooth5((q-state_cfg.preload_end_s)/(state_cfg.entry_reference_s-state_cfg.preload_end_s));
  beta=(1-h)*entry_cfg.nodes(3).x(2)+h*entry_cfg.nodes(9).x(2);
  j=3;
  for k=3:8
   if (beta-entry_cfg.nodes(k).x(2))*sign(last.x(2)-entry_cfg.nodes(3).x(2))>=0,j=k;end
  end
  f=(beta-entry_cfg.nodes(j).x(2))/(entry_cfg.nodes(j+1).x(2)-entry_cfg.nodes(j).x(2));
  ref=mixnode(entry_cfg.nodes(j),entry_cfg.nodes(j+1),min(max(f,0),1));
 end
 if phase==1
  if inside,captureTime=captureTime+Ts;else,captureTime=0;end
  % 原七项物理接管+独立600原生点确认；原生事件由快采样块生成。
  if q>=state_cfg.entry_reference_s-1e-9&&captureTime>=.3-1e-9&&stadium_packet(1)>0.5
   phase=2;mixStart=t;
  end
 end
 if phase==2
  mix=smooth5((t-mixStart)/state_cfg.handover_s);ref=mixnode(ref,last,mix);
  if t-mixStart>=state_cfg.handover_s-1e-9,phase=3;holdStart=t;end
 end
 if phase==3
  ref=last;mix=1;holdTime=max(0,t-holdStart);
  % 固定窗口满10s即退漂；不以较好的末段RMSE来推迟或重开窗口。
  if holdTime>=state_cfg.maintain_s-1e-9
   if state_cfg.bidirectional>0,phase=7;turnStart=t;else,phase=4;exitStart=t;end
   holdgate=true;
  end
 end
 weight=mix;
end
if phase>=4&&phase<=6
 exitElapsed=max(0,t-exitStart);exitProgress=min(exitElapsed/state_cfg.exit_reference_s,1);
 % 同步七节点退出：四类参数共享唯一进度。内部节点不重新减速至零。
 assert(state_cfg.exit_mode==4,'Synchronous exit mode required'); % 只允许同步七节点；代码生成诊断使用ASCII。
 substage=4;nodeProgress=smooth5(exitProgress);nodeIndex=1;
 for k=1:6,if nodeProgress>=state_cfg.exit_grid(k),nodeIndex=k;end,end
 nodeFraction=(nodeProgress-state_cfg.exit_grid(nodeIndex))/(state_cfg.exit_grid(nodeIndex+1)-state_cfg.exit_grid(nodeIndex));
 ref=mixnode(cycle_cfg.nodes(nodeIndex),cycle_cfg.nodes(nodeIndex+1),nodeFraction);
 gainProgress=exitProgress; % 同一时钟；不再调用旧K调度。
 normal=cycle_cfg.nodes(end);
 inside=all(abs(x-normal.x)<=cycle_cfg.straight_capture)&&all(Fz>0)&&~protection&&exitProgress>=1-1e-8&&gainProgress>=1-1e-8;
 if inside,straightCapture=straightCapture+Ts;else,straightCapture=0;end
 if phase==4&&straightCapture>=.3-1e-9,phase=5;straightMix=0;end
 if phase==5,straightMix=min(1,straightMix+Ts/.5);if straightMix>=1-1e-9,phase=6;end,end
 weight=0;if phase==5,weight=smooth5(straightMix);elseif phase==6,weight=1;end
 ref=mixnode(ref,normal,weight);
end
% 左右换向采用同一进度；中间12m/s直行节点不触发直行等待或状态复位。
if phase>=7
 lastR=state_cfg.right_entry_nodes(end);normalR=cycle_cfg.nodes(end);
 insideR=all(abs(x-lastR.x)<=entry_cfg.thresholds)&&all(Fz>0)&&satTime<.2;
 if phase<=9
  tau=max(0,t-turnStart);exitProgress=min(tau/state_cfg.reversal_s,1);gainProgress=exitProgress;
  nodeProgress=smooth5(exitProgress);nodeIndex=1;
  for k=1:14,if nodeProgress>=state_cfg.reversal_grid(k),nodeIndex=k;end,end
  nodeFraction=(nodeProgress-state_cfg.reversal_grid(nodeIndex))/(state_cfg.reversal_grid(nodeIndex+1)-state_cfg.reversal_grid(nodeIndex));
  ref=mixnode(state_cfg.reversal_nodes(nodeIndex),state_cfg.reversal_nodes(nodeIndex+1),nodeFraction);
  inside=insideR;exitElapsed=tau;substage=7;
  if phase==7
   if insideR,rightCapture=rightCapture+Ts;else,rightCapture=0;end
   if exitProgress>=1-1e-9&&rightCapture>=.3-1e-9&&stadium_packet(6)>.5,phase=8;rightMix=t;end
  end
  if phase==8
   mix=smooth5((t-rightMix)/state_cfg.handover_s);ref=mixnode(ref,lastR,mix);
   if t-rightMix>=state_cfg.handover_s-1e-9,phase=9;rightHold=t;end
  end
  if phase==9
   ref=lastR;holdTime=max(0,t-rightHold);
   if holdTime>=state_cfg.maintain_s-1e-9,phase=10;rightExit=t;holdgate=true;end
  end
 end
 if phase>=10
  exitElapsed=max(0,t-rightExit);exitProgress=min(exitElapsed/state_cfg.exit_reference_s,1);gainProgress=exitProgress;
  nodeProgress=smooth5(exitProgress);nodeIndex=1;substage=10;
  for k=1:6,if nodeProgress>=state_cfg.exit_grid(k),nodeIndex=k;end,end
  nodeFraction=(nodeProgress-state_cfg.exit_grid(nodeIndex))/(state_cfg.exit_grid(nodeIndex+1)-state_cfg.exit_grid(nodeIndex));
  ref=mixnode(state_cfg.right_exit_nodes(nodeIndex),state_cfg.right_exit_nodes(nodeIndex+1),nodeFraction);
  inside=all(abs(x-normalR.x)<=cycle_cfg.straight_capture)&&all(Fz>0)&&~protection&&exitProgress>=1-1e-8;
  if inside,rightStraightCapture=rightStraightCapture+Ts;else,rightStraightCapture=0;end
  if phase==10&&rightStraightCapture>=.3-1e-9,phase=11;rightStraightMix=0;end
  if phase==11,rightStraightMix=min(1,rightStraightMix+Ts/.5);if rightStraightMix>=1-1e-9,phase=12;end,end
  weight=0;if phase==11,weight=smooth5(rightStraightMix);elseif phase==12,weight=1;end
  straightCapture=rightStraightCapture;ref=mixnode(ref,normalR,weight);
 end
end
% 独立控制周期诊断：模式、参考/K时间进度、实际K列展开15项、四轮偏置。
schedule_diag=[state_cfg.exit_mode;exitProgress;gainProgress;ref.K(:);ref.offset;nodeProgress;nodeIndex;nodeFraction];
request=ref.v-feedback_delta(ref.K,x-ref.x);
rates=zeros(7,1);if refInitialized,rates=(ref.x-previousRef)/Ts;end;previousRef=ref.x;refInitialized=true;
cycle_diag=[phase;exitProgress;substage;holdTime;double(holdgate);double(inside);straightCapture;weight;exitElapsed;rates];
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
