function table=repair_build_geometry(cfg)
% 初始化及独立工况验证共用几何生成器；均匀0.01m节点，位置按切向积分。
if (cfg.mode==4 || cfg.mode==5),extent=cfg.length;else,extent=cfg.V*60+100;end
s=linspace(0,extent,ceil(extent/.01)+1)';chi=zeros(size(s));
for j=1:numel(s),[~,~,~,~,chi(j)]=repair_path_profile(s(j),cfg);end
x=cumtrapz(s,cos(chi));y=cumtrapz(s,sin(chi));
table=[s,x,y,chi];
if (cfg.mode==4 || cfg.mode==5)
 assert(norm(table(end,2:3))<1e-5,'闭合操场几何误差超过容差。');
end
end
