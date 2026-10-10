function render_comparison()
% 仅绘制原生指标配套的展示缓存；不调用初始化/模型/车辆。缓存按0.01s展示。
w=fileparts(fileparts(mfilename('fullpath')));cache=fullfile(w,'cache');out=fullfile(w,'results/figures');if ~isfolder(out),mkdir(out);end
set(groot,'defaultFigureVisible','off','defaultAxesFontName','Microsoft YaHei','defaultTextFontName','Microsoft YaHei','defaultTextInterpreter','none','defaultLegendInterpreter','none');
cases={'kf384_kr384','kf192_kr384','kf576_kr384','kf384_kr192','kf384_kr576'};names={'384/384','192/384','576/384','384/192','384/576'};colors=lines(5);phases={'entry','reversal','exit'};titles={'左向起漂：起点对齐','左→右换向：phase7对齐','最终退漂：phase10对齐'};
D=cell(5,3);C=cell(5,3);for k=1:5,for j=1:3,p=fullfile(cache,[cases{k} '_' phases{j} '.csv']);if isfile(p),D{k,j}=readmatrix(p);C{k,j}=readmatrix(fullfile(cache,[cases{k} '_' phases{j} '_control.csv']));end,end,end
titles{3}=sprintf('最终退漂：%d组完整窗口',sum(~cellfun(@isempty,D(:,3))));
f=figure('Position',[50 50 1650 1050]);tl=tiledlayout(3,3,'TileSpacing','compact');
labels={'V [m/s]','β [deg]','车体AVz [rad/s]'};
for i=1:3,for j=1:3,nexttile((i-1)*3+j);hold on;for k=1:5,d=D{k,j};if isempty(d),continue;end;plot(d(:,1),d(:,i+1),'Color',colors(k,:),'LineWidth',1.1,'DisplayName',names{k});end
 for k=1:5,d=D{k,j};if isempty(d),continue;end;plot(d(:,1),d(:,i+4),'--','Color',[.5 .5 .5],'HandleVisibility','off');end
 grid on;ylabel(labels{i});xlabel('阶段相对时间 [s]');if i==1,title(titles{j});legend('Location','best');end
end,end;title(tl,'固定控制器五刚度状态对照；灰虚线为各组实际参考；指标按0.0005 s原生数据');savefigpng(f,'states');
f=figure('Position',[50 50 1300 1050]);tl=tiledlayout(3,2,'TileSpacing','compact');
for j=2:3,for i=1:3,nexttile((i-1)*2+j-1);hold on;for k=1:5,d=D{k,j};if isempty(d),continue;end
 if i==1,y=d(:,8);elseif i==2,y=max(abs(d(:,10:11)),[],2);else,y=sqrt(mean(d(:,12:15).^2,2));end
 plot(d(:,1),y,'Color',colors(k,:),'DisplayName',names{k});end;grid on;xlabel('阶段相对时间 [s]');if i==1,ylabel('实际左前轮角 [deg]');title(titles{j});elseif i==2,ylabel('左右前轮最大|转角速率| [deg/s]');else,ylabel('四轮瞬时合并轮矩RMS [N·m]');end;if i==1&&j==2,legend;end
end,end;title(tl,'实际输入；轮矩RMS为控制负担，不是能耗');savefigpng(f,'actual_inputs');
f=figure('Position',[50 50 1400 1000]);tl=tiledlayout(3,2,'TileSpacing','compact');
for j=2:3,for i=1:3,nexttile((i-1)*2+j-1);hold on;for k=1:5,cc=C{k,j};if isempty(cc),continue;end;scale=1;if i==1,scale=180/pi;end;plot(cc(:,1),cc(:,i+7)*scale,'Color',colors(k,:),'DisplayName',names{k});plot(cc(:,1),cc(:,i+4)*scale,':','Color',colors(k,:),'HandleVisibility','off');end;grid on;xlabel('阶段相对时间 [s]');if i==1,ylabel('转向目标 [deg]');title(titles{j});elseif i==2,ylabel('前轴κ目标');else,ylabel('后轴κ目标');end;if i==1&&j==2,legend;end;end,end
title(tl,'实线：预算后目标；点线：原始请求；同一反馈下实际请求允许不同');savefigpng(f,'request_budget');
for what=1:2
 f=figure('Position',[50 50 1400 1100]);tl=tiledlayout(4,2,'TileSpacing','compact');wheels={'FL','FR','RL','RR'};
 for j=2:3,for i=1:4,nexttile((i-1)*2+j-1);hold on;for k=1:5,d=D{k,j};if isempty(d),continue;end;col=27+i;if what==2,col=15+i;end;plot(d(:,1),d(:,col),'Color',colors(k,:),'DisplayName',names{k});end;grid on;xlabel('阶段相对时间 [s]');if what==1,ylabel([wheels{i} ' Fz [N]']);else,ylabel([wheels{i} ' κ']);end;if i==1,title(titles{j});end;if i==1&&j==2,legend;end;end,end
 if what==1,title(tl,'同刻四轮轮荷；各组极值不等同同刻调载量');savefigpng(f,'wheel_loads');else,title(tl,'四轮实际纵向滑移率；目标相同不等于实际滑移相同');savefigpng(f,'wheel_slip');end
end
f=figure('Position',[50 50 1400 1000]);tl=tiledlayout(4,2,'TileSpacing','compact');
for j=2:3,for i=1:4,nexttile((i-1)*2+j-1);hold on;for k=1:5,d=D{k,j};if isempty(d),continue;end;plot(d(:,1),d(:,31+i),'Color',colors(k,:),'DisplayName',names{k});end;grid on;xlabel('阶段相对时间 [s]');lab={'前Aux力矩 [N·m]','后Aux力矩 [N·m]','前相对侧倾角 [deg]','后相对侧倾角 [deg]'};ylabel(lab{i});if i==1,title(titles{j});end;if i==1&&j==2,legend;end;end,end;title(tl,'内部Aux刚度作用：M=Kφ；外部ASB全零');savefigpng(f,'aux_roll');
f=figure('Position',[50 50 1800 1150]);tl=tiledlayout(4,4,'TileSpacing','compact');wheels={'FL','FR','RL','RR'};
for i=1:4,for j=1:4,nexttile((i-1)*4+j);hold on;stage=2+(j>2);isfy=mod(j,2)==0;
 for k=1:5,d=D{k,stage};if isempty(d),continue;end;col=19+i+4*isfy;plot(d(:,1),d(:,col),'Color',colors(k,:),'DisplayName',names{k});end
 grid on;xlabel('阶段相对时间 [s]');if isfy,ylabel([wheels{i} ' Fy [N]']);else,ylabel([wheels{i} ' Fx [N]']);end;if i==1,title(titles{stage});end;if i==1&&j==1,legend;end
end,end;title(tl,'四轮Fx/Fy保持原轮胎坐标；不直接当作整轴车体系合力');savefigpng(f,'tire_forces');
f=figure('Position',[50 50 1100 850]);hold on;for k=1:5,p=fullfile(cache,[cases{k} '_trajectory.csv']);d=readmatrix(p);ix=d(:,4)>=1&d(:,4)<10;plot(d(ix,2),d(ix,3),'Color',colors(k,:),'DisplayName',names{k},'LineWidth',1.2);if max(d(:,4))<10,plot(d(end,2),d(end,3),'x','Color',colors(k,:),'MarkerSize',13,'LineWidth',2,'HandleVisibility','off');end;end;axis equal;grid on;xlabel('X [m]');ylabel('Y [m]');legend('Location','best');title({'五种刚度自然轨迹：起漂至最终退漂开始或异常终止','本批统一直线道路坐标；自然轨迹不考核几何闭合'});savefigpng(f,'trajectories');
f=figure('Position',[50 50 1550 650]);axis off;
annotation(f,'textbox',[.05 .84 .9 .12],'String',{'Step37c 实际系统结构：道路坐标修复后五刚度对照','五种配置共用Step37a LeftRight_R3；不按刚度重配平或调增益'},'Interpreter','none','EdgeColor','none','FontSize',17,'HorizontalAlignment','center');
texts={{'普通GUI初态','左漂→连续换向','右漂→自然直行'},{'冻结同步调度','x参考 / 前馈','K(3×5) / 轮矩偏置'},{'五状态反馈','[V β AVz κf κr]','请求：δ κf κr'},{'冻结公共执行层','Fx补偿 / 轮惯量','限速 / 轮速 / 轮矩预算'},{'CarSim车辆','统一直线道路坐标','轮胎 / 悬架 / Aux'}};
for k=1:5,x0=.025+(k-1)*.198;color=[.91 .95 .99];if k==5,color=[1 .91 .75];end;annotation(f,'textbox',[x0 .47 .175 .23],'String',texts{k},'Interpreter','none','FitBoxToText','off','BackgroundColor',color,'FontSize',12,'HorizontalAlignment','center','VerticalAlignment','middle');if k<5,annotation(f,'arrow',[x0+.175 x0+.197],[.585 .585]);end;end
annotation(f,'line',[.9 .9],[.47 .34]);annotation(f,'line',[.9 .5],[.34 .34]);annotation(f,'arrow',[.5 .5],[.34 .47]);
annotation(f,'textbox',[.30 .19 .64 .12],'String',{'实际状态反馈；控制0.01 s，CarSim/实际输入0.0005 s','外部ASB四路全零；无路径控制；XY只用于自然轨迹诊断'},'Interpreter','none','EdgeColor','none','FontSize',13,'HorizontalAlignment','center');
annotation(f,'textbox',[.05 .04 .9 .10],'String','橙色框：仅道路参考坐标改为直线；Z=0、μ=0.85、RR=1保持；五组间仅Kf/Kr不同。','Interpreter','none','EdgeColor','none','FontSize',12,'HorizontalAlignment','center');
print(f,fullfile(out,'system_structure.svg'),'-dsvg');savefigpng(f,'system_structure');
disp('STEP37B_PLOTS_DONE');
function savefigpng(fig,name),exportgraphics(fig,fullfile(out,[name '.png']),'Resolution',140);close(fig);end
end
