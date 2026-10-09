function [forces,diag]=asb_pulse_force(raw,phi,rate,moment,dk,asb_cfg,dm)
%#codegen
% 轮端力偶到弹簧附加力 / Wheel Couple to Spring Force。
% 内部轮序FL FR RL RR；GUI排列仍由原Selector转为L1 L2 R1 R2。
% 原物理映射不变：正FsExt为弹簧压缩力，运动比前0.959、后0.797。
% q=M/lever，左右轮端分别+q/-q；每轴轮端合力为零。
lever=asb_cfg.track_m-[sum(raw(21:22));sum(raw(23:24))]*.001;
assert(all(lever>0)&&all(isfinite(moment)),'ASB:Invalid force-couple geometry');
q=moment./lever;wheel=[q(1);-q(1);q(2);-q(2)];forces=wheel./asb_cfg.motion_ratio;
% 实际接收的FsExt用于回查，不能用指令冒充车辆收到的力矩。
actual=raw(5:8);actualWheel=actual.*asb_cfg.motion_ratio;
actualMoment=[(actualWheel(1)-actualWheel(2))*lever(1)/2;(actualWheel(3)-actualWheel(4))*lever(2)/2];
net=[sum(actualWheel(1:2));sum(actualWheel(3:4))];
power=moment.*rate;jrate=raw(13:16)*.001;
powerSpring=-[sum(actualWheel(1:2).*jrate(1:2));sum(actualWheel(3:4).*jrate(3:4))];
geometricPhi=atan2([raw(10)-raw(9);raw(12)-raw(11)]*.001,lever);
% 保持44路布局；35:36改为真实开环力矩指令导数dm，3:4仍为零辅助刚度。
diag=[phi;rate;dk;moment;actualMoment;actualMoment-moment;lever;wheel;forces;actual;net;power;powerSpring;powerSpring-power;dm;geometricPhi;geometricPhi-phi;raw(25:26);asb_cfg.target_K_deg];
end
