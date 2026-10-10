function render_road_diagnosis()
% 只画道路坐标几何及已记录的报错查询点；不把查询点误认为车辆质心。
w=fileparts(fileparts(mfilename('fullpath')));out=fullfile(w,'results/figures');if ~isfolder(out),mkdir(out);end
set(groot,'defaultFigureVisible','off','defaultAxesFontName','Microsoft YaHei','defaultTextFontName','Microsoft YaHei','defaultTextInterpreter','none');
r=jsondecode(fileread(fullfile(w,'audit/road_diagnosis.json')));
f=figure('Position',[50 50 1350 650]);tl=tiledlayout(1,2,'TileSpacing','compact');
nexttile;hold on;a=linspace(0,2*pi,501);plot(15*sin(a),-1-15*cos(a),'Color',[.5 .5 .5],'DisplayName','旧道路参考圆 R=15 m');plot([-30 30],[-16 -16],'b--','DisplayName','新道路参考直线 Y=-16 m');plot(0,-1,'kx','LineWidth',2,'DisplayName','旧圆心 (0,-1)');
colors=lines(2);n=0;
for i=1:numel(r.cases)
 item=onecase(r.cases,i);if ~isfield(item,'error')||isempty(item.error),continue;end
 n=n+1;e=item.error;plot(e.query_X,e.query_Y,'o','Color',colors(n,:),'DisplayName',item.case,'MarkerFaceColor',colors(n,:));
end
axis equal;grid on;xlabel('X [m]');ylabel('Y [m]');legend('Location','best','Interpreter','none');title('仅更换道路参考坐标；物理路面仍为均匀平面');
nexttile;hold on;plot(0,-1,'kx','LineWidth',2,'DisplayName','旧圆心');n=0;
for i=1:numel(r.cases)
 item=onecase(r.cases,i);if ~isfield(item,'error')||isempty(item.error),continue;end
 n=n+1;e=item.error;plot(e.query_X,e.query_Y,'o','Color',colors(n,:),'MarkerFaceColor',colors(n,:),'DisplayName',item.case);plot([0 e.query_X],[-1 e.query_Y],':','Color',colors(n,:),'HandleVisibility','off');text(e.query_X+.012,e.query_Y,sprintf('距圆心 %.3f m\nT=%.4f s',e.center_distance_m,e.t_s),'FontSize',11);
end
assert(n==2,'道路诊断图必须包含两个真实错误查询点');
axis equal;xlim([-.1 .23]);ylim([-1.24 -.95]);grid on;xlabel('X [m]');ylabel('Y [m]');title('旧报错查询点：非已确认的质心或特定车轮');
title(tl,'圆心附近 S-L 变换条件恶化；报错点未恰好位于数学奇点');exportgraphics(f,fullfile(out,'road_diagnosis.png'),'Resolution',150);close(f);
end
function item=onecase(c,i)
% JSON异构对象数组在MATLAB中解码为cell；必须取对象而非cell容器。
if iscell(c),item=c{i};else,item=c(i);end
end
