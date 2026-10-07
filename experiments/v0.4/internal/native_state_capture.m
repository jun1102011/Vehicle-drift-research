function [packet,stadium_diag,roll_feedback,feedback_diag]=stadium_guidance(y,t,stadium_cfg,entry_cfg,repair_cfg,state_cfg)
%#codegen
% 原操场模块现为独立原生事件检查：0.0005s每点计算，不投影/纠偏。
% packet19仅前5项有意义：已确认、确认时刻、合格段起点、计数、当前合格。
% 通过0.01s采样传给阶段调度；600点事件时刻仍保存原生精度。
% 旧输出尺寸保持：stadium_diag17全零，roll_feedback6物理诊断，feedback_diag17。
persistent count latched confirm eventStart lastIndex
if isempty(count),count=0;latched=false;confirm=-1;eventStart=-1;lastIndex=-1;end
packet=zeros(19,1);stadium_diag=zeros(17,1);roll_feedback=zeros(6,1);feedback_diag=zeros(17,1);
x=[hypot(y(2),y(3))/3.6;y(4)*pi/180;y(5)*pi/180];
inside=all(abs(x-state_cfg.target)<=state_cfg.entry_tolerance)&&x(3)>0;
index=round(t/state_cfg.native_step_s);
if index>lastIndex
 assert(index==lastIndex+1,'Native entry event missed 0.0005s sample');
 if inside,count=count+1;else,count=0;end
 if count==state_cfg.entry_samples&&~latched
  latched=true;confirm=t;eventStart=t-(state_cfg.entry_samples-1)*state_cfg.native_step_s;
 end
 lastIndex=index;
end
packet(1:5)=[double(latched);confirm;eventStart;count;double(inside)];
roll_feedback=[y(8)*pi/180;y(26)*pi/180;0;0;0;0];
feedback_diag(1:12)=[count;double(inside);double(latched);eventStart;confirm;index;x;state_cfg.target];
end
