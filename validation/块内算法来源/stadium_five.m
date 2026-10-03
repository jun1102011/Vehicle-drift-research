function [packet,stadium_diag,roll_feedback,feedback_diag]=stadium_guidance(y,t,stadium_cfg, entry_cfg, repair_cfg)
%#codegen
% 固定路线投影→阶段/局部工作点→小幅横摆修正；不直接驱动车轮。
% y：SI换算前CarSim50路；packet：参考状态7、前馈3、负载偏置4、请求3、阶段、进度。
% stadium_diag：s,bend,phase,ey,dChi,原始dr,限后dr,r_ref,r_modified,两限幅标志,
% 局部XY,chi_ref,gain,lap,切换失败。位置m，角度rad，角速度rad/s。
% 路线来源于已验收单弯，在试验前冻结；80m引入段不计入闭合路线。
persistent initialized previous chi j dr previousPhase failed failedPlanar failedRoll failedContact
feedback_diag=zeros(17,1);roll_feedback=zeros(6,1);packet=zeros(19,1);stadium_diag=zeros(17,1);
if repair_cfg.stadium_mode==0,return;end
c=stadium_cfg;h=c.angle;R=[cos(h),sin(h);-sin(h),cos(h)];
if isempty(initialized),previous=y(35:36);chi=(y(37)+y(4))*pi/180;j=1;dr=0;previousPhase=0;failed=false;failedPlanar=false;failedRoll=false;failedContact=false;initialized=true;end
point=R*(y(35:36)-c.origin);delta=y(35:36)-previous;
if norm(delta)>1e-8,chi=atan2(delta(2),delta(1));end;previous=y(35:36);
% 只在上一投影附近搜索，防止跳到对向直线。线段投影是有限区间内的最近点。
n=numel(c.s);low=max(1,j-20);high=min(n-1,j+39);best=inf;chosen=j;fraction=0;closest=zeros(2,1);
for k=low:high
 a=c.xy(k,:)';d=c.xy(k+1,:)'-a;f=min(max(dot(point-a,d)/dot(d,d),0),1);near=a+f*d;distance=sum((point-near).^2);
 if distance<best,best=distance;chosen=k;fraction=f;closest=near;end
end
j=chosen;f=fraction;s=(1-f)*c.s(j)+f*c.s(j+1);chiref=(1-f)*c.chi(j)+f*c.chi(j+1);
direction=R*[cos(chi);sin(chi)];localchi=atan2(direction(2),direction(1));dchi=atan2(sin(localchi-chiref),cos(localchi-chiref));
ey=dot(point-closest,[-sin(chiref);cos(chiref)]);
refx=((1-f)*c.x(j,:)+f*c.x(j+1,:))';refv=((1-f)*c.v(j,:)+f*c.v(j+1,:))';offset=((1-f)*c.offset(j,:)+f*c.offset(j+1,:))';
K=zeros(3,5);for a=1:3,for b=1:5,K(a,b)=(1-f)*c.K(j,a,b)+f*c.K(j+1,a,b);end,end
w=1; % 编译时显式定义初始混合权重，t>=2不混合。
if t<2
 q=min(max(t/2,0),1);w=10*q^3-15*q^4+6*q^5;
 refx=(1-w)*entry_cfg.normal.x+w*refx;refv=(1-w)*entry_cfg.normal.v+w*refv;offset=(1-w)*entry_cfg.normal.offset+w*offset;K=(1-w)*entry_cfg.normal.K+w*K;
end
% 行进方向反馈不是路径LQR。正ey/正方向误差时要求减少左转。
gain=c.g0+(c.g1-c.g0)*min(abs(refx(2))/(10*pi/180),1);
rawdr=-(c.wn^2/max(refx(1),1)*ey+2*c.wn*dchi)/gain;
target=min(max(rawdr,-c.correction_limit),c.correction_limit);dr=min(max(target,dr-c.correction_rate*repair_cfg.Ts),dr+c.correction_rate*repair_cfg.Ts);
x=[hypot(y(2),y(3))/3.6;y(4)*pi/180;y(5)*pi/180;mean(y(27:28));mean(y(29:30));y(8)*pi/180;y(26)*pi/180];
% 五项误差反馈与小幅横摆修正共用当前冻结K，不引入第二套执行命令。
% Roll/AVx仍记录并参与原接管门控；其连续反馈贡献固定为零。
request=refv-K*(x(1:5)-refx(1:5))+K(:,3)*dr;
roll_feedback=zeros(6,1); % 保留日志端口兼容，明确两项直接反馈均为零。
phase=c.phase(j);
% 位置到达但状态未就绪时记录任务失败，不推迟路线或重置车辆掩盖偏差。
if phase~=previousPhase
 if phase==3,failed=failed||any(abs(x-entry_cfg.nodes(end).x)>entry_cfg.thresholds)||any(y(17:20)<=0);end
 if phase==5,limit=[.2;pi/180;.02;.02;.04;2*pi/180;10*pi/180];failed=failed||any(abs(x-refx)>limit)||any(y(17:20)<=0);end
 % 原失败标志保留；新增分项使Roll偏差不会被误报为横纵向失效。
 if phase==3
  err=abs(x-entry_cfg.nodes(end).x);lim=entry_cfg.thresholds;
  failedPlanar=failedPlanar||any(err(1:5)>lim(1:5));
  failedRoll=failedRoll||any(err(6:7)>lim(6:7));
  failedContact=failedContact||any(y(17:20)<=0);
 elseif phase==5
  err=abs(x-refx);lim=[.2;pi/180;.02;.02;.04;2*pi/180;10*pi/180];
  failedPlanar=failedPlanar||any(err(1:5)>lim(1:5));
  failedRoll=failedRoll||any(err(6:7)>lim(6:7));
  failedContact=failedContact||any(y(17:20)<=0);
 end
end
% 前7项记录反馈维数与两项零贡献，随后为名义前馈、实际请求及失败分项。
feedback_diag=[5;zeros(6,1);refv;request;double(failedPlanar);double(failedRoll);double(failedContact);double(failed)];
previousPhase=phase;packet=[refx;refv;offset;request;phase;s/c.length];
stadium_diag=[s;c.bend(j);phase;ey;dchi;rawdr;dr;refx(3);refx(3)+dr;double(abs(rawdr)>c.correction_limit);double(abs(dr-target)>1e-9);point;chiref;gain;double(s>=c.length-1e-7);double(failed)];
end
