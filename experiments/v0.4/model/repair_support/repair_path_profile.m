function [k,bw,dbds,stage,chi]=repair_path_profile(s,cfg)
% 曲率与侧偏角分别生成；chi是曲率对弧长的解析积分。
% mode=1入漂维持，2同步退出，3先恢复beta再退出，4原操场，5提前恢复beta操场。
L=cfg.transition;R=cfg.R;C=cfg.plateau;
% 普通轨迹：stage=1直线、2入弯、3恒曲率、4出弯、5末直线。
% 曲率与漂移权重独立；mode=6始终bw=0，不能只把beta设零。
if cfg.mode==6
 q=s-cfg.straight;bw=0;dbds=0;
 if q<=0,k=0;chi=0;stage=1;
 elseif q<L
  u=q/L;[h,~]=smooth(u);k=h/R;chi=L/R*primitive(u);stage=2;
 elseif q<L+C,k=1/R;chi=(q-L/2)/R;stage=3;
 elseif q<2*L+C
  u=(q-L-C)/L;[h,~]=smooth(u);k=(1-h)/R;chi=(L/2+C+L*(u-primitive(u)))/R;stage=4;
 else,k=0;chi=(L+C)/R;stage=5;end
 return;
end
if cfg.mode==4 || cfg.mode==5
 q=mod(s,cfg.half)-cfg.straight;half=floor(s/cfg.half);
 if q<=0,k=0;bw=0;dbds=0;stage=1;theta=0;
 elseif q<L
  u=q/L;[bw,db]=smooth(u);dbds=db/L;k=bw/R;stage=2;theta=L/R*primitive(u);
 elseif q<L+C
  k=1/R;bw=1;dbds=0;stage=3;theta=(q-L/2)/R;
  if cfg.mode==5 && q>L+C-L
   u=(q-(L+C-L))/L;[z,dz]=smooth(u);bw=1-z;dbds=-dz/L;stage=4;
  end
 else
  u=min((q-L-C)/L,1);[z,dz]=smooth(u);k=(1-z)/R;
  if cfg.mode==4,bw=1-z;dbds=-dz/L;else,bw=0;dbds=0;end
  stage=4;theta=(L/2+C+L*(u-primitive(u)))/R;
 end
 chi=pi*half+theta;stage=stage+4*mod(half,2);return;
end
% 入漂维持允许曲率与侧偏的建立长度/起点不同；chi仍只由曲率积分确定。
if cfg.mode==1
 q=s-cfg.straight;u=min(max(q/cfg.transition,0),1);[hk,~]=smooth(u);k=hk/cfg.R;
 if q<=0,chi=0;elseif q<cfg.transition,chi=cfg.transition/cfg.R*primitive(u);else,chi=(q-cfg.transition/2)/cfg.R;end
 v=min(max((q-cfg.beta_delay)/cfg.beta_transition,0),1);[bw,db]=smooth(v);dbds=db/cfg.beta_transition;
 if q<=0,stage=1;elseif u<1||v<1,stage=2;else,stage=3;end
 return;
end
q=s-cfg.straight;
if q<=0,k=0;bw=0;dbds=0;stage=1;chi=0;return;end
if q<L
 u=q/L;[bw,db]=smooth(u);dbds=db/L;k=bw/R;stage=2;chi=L/R*primitive(u);return;
end
k=1/R;bw=1;dbds=0;stage=3;chi=(q-L/2)/R;
if cfg.mode==1,return;end
% 独立退出站点由已通过3s稳态门槛的前置试验给出，不猜测稳定起点。
e=s-cfg.exit_s;if e<0,return;end
if cfg.mode==2
 u=min(e/L,1);[z,dz]=smooth(u);k=(1-z)/R;bw=1-z;dbds=-dz/L;
 chi=(cfg.exit_s-cfg.straight-L/2+L*(u-primitive(u)))/R;stage=4;
else
 u=min(e/L,1);[z,dz]=smooth(u);bw=1-z;dbds=-dz/L;stage=4;
 if e>=L
  z2=min((e-L)/L,1);[h,~]=smooth(z2);k=(1-h)/R;
  chi=(cfg.exit_s+L-cfg.straight-L/2+L*(z2-primitive(z2)))/R;stage=5;
 end
end
end
function [h,dh]=smooth(u)
h=10*u^3-15*u^4+6*u^5;dh=30*u^2*(1-u)^2;
end
function v=primitive(u)
v=2.5*u^4-3*u^5+u^6;
end
