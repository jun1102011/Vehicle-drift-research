function [f,diag]=asb_force(y)
%#codegen
% 理想固定刚度稳定杆 / Ideal Fixed-Stiffness ASB
% phi为各轴悬架相对侧倾deg，不以整车Roll替代。
% M=-DeltaK*phi，DeltaK单位Nm/deg；F_spring=(+M/lever,-M/lever)/MR。
% 力臂、运动比沿用已验证接口；输出按GUI L1,L2,R1,R2排列。
lever=1.675-[sum(y(71:72));sum(y(73:74))]*.001;
assert(all(lever>0)&&all(isfinite(y)));
M=-deltaK.*y(51:52);q=M./lever;
wheel=[q(1);-q(1);q(2);-q(2)]./[.959;.959;.797;.797];
f=wheel([1,3,2,4]);power=M.*(y(53:54)*pi/180);
diag=[M;lever;wheel;power];
end
