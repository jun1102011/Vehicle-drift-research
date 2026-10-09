function [phi,rate,moment,dk,dm,pulse_diag]=asb_pulse_moment(raw,asb_cfg,t,stage)
%#codegen
% 起漂相对时间单轴脉冲 / Entry-relative Single-axis Pulse。
% stage为原cycle_diag，只有phase用于首次起漂锁存；不反馈beta或轮荷。
% 正力矩：左正/右负轮端力偶。phi/rate仅机械诊断，不参与输入决策。
persistent entryTime latched
if isempty(latched),latched=false;entryTime=0;end
if ~latched && stage(1)==1,entryTime=t;latched=true;end
assert(all(isfinite(raw)),'ASB:Nonfinite suspension measurement');
phi=raw(1:2)*pi/180;rate=raw(3:4)*pi/180;dk=zeros(2,1);
tau=0;if latched,tau=t-entryTime;end
g=0;dg=0;lo=asb_cfg.pulse_start_s;hi=asb_cfg.pulse_end_s;edge=asb_cfg.pulse_edge_s;
if latched && tau>lo && tau<hi
 if tau<lo+edge
  q=(tau-lo)/edge;g=(1-cos(pi*q))/2;dg=pi*sin(pi*q)/(2*edge);
 elseif tau>hi-edge
  q=(hi-tau)/edge;g=(1-cos(pi*q))/2;dg=-pi*sin(pi*q)/(2*edge);
 else,g=1;end
end
moment=asb_cfg.pulse_moment_Nm*g;dm=asb_cfg.pulse_moment_Nm*dg;
if all(asb_cfg.pulse_moment_Nm==0),dm=zeros(2,1);end % 零回归保留原诊断+0位模式。
% 独立12项诊断：锁存、起点、tau、窗口标志、包络及导数、两轴力矩/导数、phase、归一化进度。
pulse_diag=[double(latched);entryTime;tau;double(latched&&tau>=lo&&tau<hi);g;dg;moment;dm;stage(1);min(max((tau-lo)/(hi-lo),0),1)];
end
