function render_closure
% 只读取Step35n绘图缓存并输出关键图；不初始化模型、不启动CarSim。
W=fileparts(fileparts(mfilename('fullpath'))); C=fullfile(W,'cache'); F=fullfile(W,'results','figures'); if ~isfolder(F),mkdir(F);end
S=readtable(fullfile(W,'results','summary.csv'),'VariableNamingRule','preserve');
names=string(S.configuration); ids=string(S.case); colors=lines(5);
set(groot,'defaultAxesFontName','Microsoft YaHei','defaultTextFontName','Microsoft YaHei');
X=cell(5,1);P=X;U=X;B=X;Z=X;
for i=1:5
 X{i}=readmatrix(fullfile(C,ids(i)+"_states.csv")); P{i}=readmatrix(fullfile(C,ids(i)+"_physical.csv"));
 U{i}=readmatrix(fullfile(C,ids(i)+"_input.csv")); B{i}=readmatrix(fullfile(C,ids(i)+"_constraints.csv")); Z{i}=readmatrix(fullfile(C,ids(i)+"_stops.csv"));
end
f=figure('Visible','off','Position',[50 50 1450 1050]); tl=tiledlayout(3,2,'TileSpacing','compact');
labs={'V [m/s]','β [deg]','AVz [rad/s]'};targets=[12 0 0];tol=[.2 1 .02];
for k=1:3
 for zoom=1:2
  ax=nexttile;hold(ax,'on');
  for i=1:5,plot(X{i}(:,1),X{i}(:,k+1),'Color',colors(i,:),'LineWidth',1.25,'DisplayName',names(i));end
  plot(X{1}(:,1),X{1}(:,k+4),'k--','LineWidth',1.2,'DisplayName','共同参考');
  yline(targets(k)+tol(k),':','Color',[.5 .5 .5],'HandleVisibility','off');yline(targets(k)-tol(k),':','Color',[.5 .5 .5],'HandleVisibility','off');
  for i=1:5,xline(S.recovery_s(i),':','Color',colors(i,:),'HandleVisibility','off');end
  if zoom==1,xlim([0 15]);else,xlim([5.8 8.5]);ylim([targets(k)-2*tol(k),targets(k)+2*tol(k)]);end
  ylabel(labs{k});xlabel('退漂相对时间 [s]');grid on;if k==1&&zoom==1,legend('Location','best');end
 end
end
title(tl,'固定任务状态与持续恢复事件：右列为恢复段放大；虚线为参考，竖线为合格段起点');savefigs(f,F,'01_recovery');
f=figure('Visible','off','Position',[50 50 1400 850]);tl=tiledlayout(2,3,'TileSpacing','compact');
vals={S.torque_RMS_Nm,S.speed_loss_mps,S.minFz_N,S.J_final_beta,S.J_final_AVz,S.front_slip_clip_total_s};
yl={'轮矩合并RMS [N·m]','最大速度损失 [m/s]','最低轮荷 [N]','最终目标Jβ [deg²·s]','最终目标JAVz [(rad/s)²·s]','前滑移幅值裁剪 [s]'};
for k=1:6,nexttile;hold on;for i=1:5,scatter(S.recovery_s(i),vals{k}(i),65,colors(i,:),'filled');text(S.recovery_s(i)+.0006,vals{k}(i),names(i),'FontSize',9);end;xlim([6.475 6.552]);span=max(vals{k})-min(vals{k});ylim([min(vals{k})-.10*span,max(vals{k})+.10*span]);xlabel('持续恢复合格段起点 [s]');ylabel(yl{k});grid on;end
title(tl,'恢复速度与代价：不作加权评分，轮矩RMS不是能耗');savefigs(f,F,'02_tradeoffs');
f=figure('Visible','off','Position',[50 50 1500 1050]);tl=tiledlayout(3,2,'TileSpacing','compact');
for type=1:2
 nexttile;hold on;
 for i=1:5
  E=jsondecode(fileread(fullfile(W,'results',ids(i)+"_events.json")));
  if type==1,key='actual_steering_rate';else,key='front_slip_amplitude_clip';end
  for j=1:numel(E),if strcmp(E(j).constraint,key),plot([E(j).start_s E(j).end_s],[i i],'Color',colors(i,:),'LineWidth',8);end;end
 end
 yticks(1:5);yticklabels(names);ylim([.5 5.5]);xlim([0 8]);grid on;xlabel('退漂相对时间 [s]');
 if type==1,title('实际转向速率贴限（原生段，包含短暂间断）');else,title('前轴滑移幅值裁剪（0.01 s诊断）');end
end
cols=[2 3 5 6]; lbl={'移动转向边界最小余量 [deg]','前轴滑移幅值最小余量','目标轮速加速度余量 [rad/s²]','轮矩预算余量 [N·m]'};
for k=1:4,nexttile;hold on;for i=1:5,plot(B{i}(:,1),B{i}(:,cols(k)),'Color',colors(i,:));end;yline(0,'k:');grid on;xlim([0 15]);xlabel('退漂相对时间 [s]');ylabel(lbl{k});end
title(tl,'约束参与与余量：触限不等于失效；轮速余量仅指目标加速度分量');savefigs(f,F,'03_constraints');
f=figure('Visible','off','Position',[50 50 1450 1050]);tl=tiledlayout(3,2,'TileSpacing','compact');
wheels={'FL','FR','RL','RR'};
for wheel=1:4,nexttile;hold on;for i=1:5,plot(P{i}(:,1),P{i}(:,wheel+1),'Color',colors(i,:));end;yline(0,'k:');title(['轮荷 ',wheels{wheel}]);ylabel('Fz [N]');xlabel('退漂相对时间 [s]');grid on;end
nexttile;hold on;for i=1:5,plot(P{i}(:,1),P{i}(:,21),'Color',colors(i,:));end;yline(50,'k--');ylabel('后右弹簧查询 [mm]');xlabel('退漂相对时间 [s]');title('50 mm为表域上界，不是已验证机械极限');grid on;
nexttile;hold on;for i=1:5,plot(Z{i}(:,1),max(abs(Z{i}(:,2:end)),[],2),'Color',colors(i,:),'DisplayName',names(i));end;ylabel('八项止挡力最大绝对值 [N]');xlabel('退漂相对时间 [s]');title('VS/VSB原0.01 s证据');grid on;legend('Location','best');
title(tl,'轮荷与悬架代价：普通初态运行形成的退出初态不重置');savefigs(f,F,'04_load_suspension');
f=figure('Visible','off','Position',[50 50 1550 1050]);tl=tiledlayout(3,4,'TileSpacing','compact'); wheels={'FL','FR','RL','RR'};
for k=1:3,for j=1:4,nexttile;hold on;for i=1:5,plot(P{i}(:,1),P{i}(:,1+4*k+j),'Color',colors(i,:));end;title(wheels{j});if k==1,ylabel('实际κ');elseif k==2,ylabel('Fx [N]');else,ylabel('Fy [N]');end;grid on;xlim([0 15]);xlabel('τ [s]');end;end
lg=legend(names,'Orientation','horizontal');lg.Layout.Tile='south';title(tl,'四轮实际滑移及轮胎力：Fx/Fy保留原轮胎坐标，不冒充车体系合力');savefigs(f,F,'05_tires');
f=figure('Visible','off','Position',[50 50 1450 1050]);tl=tiledlayout(4,2,'TileSpacing','compact');
for k=1:6,nexttile;hold on;for i=1:5,if k<=2,v=U{i}(:,k+1);else,v=U{i}(:,k+3);end;plot(U{i}(:,1),v,'Color',colors(i,:));end;grid on;xlim([0 15]);xlabel('τ [s]');if k<=2,ylabel('车轮转角 [deg]');title(wheels{k});else,ylabel('轮矩 [N·m]');title(wheels{k-2});end;end
for k=1:2,nexttile;hold on;for i=1:5,plot(P{i}(:,1),P{i}(:,17+k),'Color',colors(i,:));end;xlim([0 15]);grid on;xlabel('τ [s]');ylabel('Aux力矩 [N·m]');if k==1,title('前轴');else,title('后轴');end;end
lg=legend(names,'Orientation','horizontal');lg.Layout.Tile='south';title(tl,'实际两前轮转角、四轮轮矩与内部Aux力矩；外部ASB四路为零');savefigs(f,F,'06_actual_inputs');
f=figure('Visible','off','Position',[50 50 1550 730],'Color','white');
annotation(f,'textbox',[.02 .9 .96 .08],'String','Step35n 实际系统：执行系统未改；本轮只新增离线收束评价','EdgeColor','none','FontSize',19,'HorizontalAlignment','center');
boxes={[.025 .55 .21 .26],[.28 .55 .20 .26],[.53 .55 .21 .26],[.79 .55 .19 .26],[.28 .17 .46 .22]};
texts={sprintf('阶段与同步七节点调度\nSchedule\n参考 / 前馈 / 3×5 K / 四轮偏置\n同一进度，T=43/6 s'),sprintf('五状态局部反馈\nFive-state Feedback\n[V, β, AVz, κf, κr]\n三路请求 δ / κf / κr'),sprintf('公共执行层\nCommon Execution\nFx补偿、轮惯量、限速\n轮速预算、±3000 N·m'),sprintf('CarSim车辆\nVehicle\n内部Aux五种刚度\n轮胎 / 悬架 / 外推'),sprintf('本轮新增：只读原始记录的离线评价 / Offline Evaluation\n恢复事件、过程误差、控制与轮荷代价、约束事件与余量\n不反馈车辆，不新增仿真，不复制raw')};
for k=1:5,co=[.9 .94 .98];if k==5,co=[1 .95 .78];end;annotation(f,'textbox',boxes{k},'String',texts{k},'BackgroundColor',co,'FontSize',13,'HorizontalAlignment','center','VerticalAlignment','middle','LineWidth',1.2);end
for x=[.235 .48 .74],annotation(f,'arrow',[x x+.045],[.68 .68],'LineWidth',1.4);end
annotation(f,'line',[.88 .88],[.55 .45],'LineWidth',1.2);annotation(f,'line',[.88 .38],[.45 .45],'LineWidth',1.2);annotation(f,'arrow',[.38 .38],[.45 .55],'LineWidth',1.2);
annotation(f,'textbox',[.38 .405 .48 .04],'String','实际状态反馈；Roll/AVx仍保留物理与诊断','EdgeColor','none','HorizontalAlignment','center','FontSize',12);
annotation(f,'line',[.88 .88],[.45 .28],'LineStyle','--');annotation(f,'arrow',[.88 .74],[.28 .28],'LineStyle','--');
annotation(f,'textbox',[.035 .035 .93 .085],'String','无路径控制；外部ASB四路恒零。控制0.01 s / CarSim及转向0.0005 s。输出是模型内闭环对照，不是实车安全或主动ASB收益。','EdgeColor','none','FontSize',12,'HorizontalAlignment','center');
exportgraphics(f,fullfile(F,'07_actual_structure.png'),'Resolution',150);saveas(f,fullfile(F,'07_actual_structure.svg'));close(f);
disp('STEP35N_FIGURES_COMPLETE');
end
function savefigs(f,F,name)
exportgraphics(f,fullfile(F,[name '.png']),'Resolution',150);close(f);
end
