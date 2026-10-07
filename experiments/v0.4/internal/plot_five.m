function plot_five()
% 只读本批4份和历史基线1份raw；不启动仿真。
w=fileparts(fileparts(mfilename('fullpath')));froot=fullfile(fileparts(w),'step35f_continuous_entry_20261007');out=fullfile(w,'results','analysis','figures');if ~isfolder(out),mkdir(out);end
cases={'kf384_kr384','kf192_kr384','kf576_kr384','kf384_kr192','kf384_kr576'};labels={'384/384','192/384','576/384','384/192','384/576'};colors=[.08 .12 .18;.08 .40 .68;.8 .36 .19;.12 .53 .38;.53 .31 .7];
set(groot,'defaultFigureVisible','off','defaultAxesFontName','Microsoft YaHei','defaultTextFontName','Microsoft YaHei','defaultAxesFontSize',11,'defaultTextInterpreter','none','defaultLegendInterpreter','none');Z=cell(5,1);
for k=1:5
 root=w;if k==1,root=froot;end
 z=load(fullfile(root,'results','runs',['V12_beta30_state_only_R3_' cases{k}],'raw_numeric.mat'),'repair_plant_raw','repair_entry_diag','repair_cycle_diag','repair_actuator_command','asb_suspension');
 z.start=z.repair_entry_diag(find(z.repair_entry_diag(:,2)==1,1),1);h=find(z.repair_cycle_diag(:,2)==3,1);x=find(z.repair_cycle_diag(:,2)==4,1);z.hold=NaN;z.exit=NaN;if ~isempty(h),z.hold=z.repair_cycle_diag(h,1);end;if ~isempty(x),z.exit=z.repair_cycle_diag(x,1);end;Z{k}=z;
end
f=figure('Position',[20 20 1280 620]);ax=axes(f);hold(ax,'on');
for k=1:5,z=Z{k};p=z.repair_plant_raw;ix=1:20:size(p,1);plot(ax,p(ix,1)-z.start,p(ix,5),'Color',colors(k,:),'LineWidth',1.8,'DisplayName',labels{k});end
z=Z{1};plot(ax,z.repair_entry_diag(:,1)-z.start,rad2deg(z.repair_entry_diag(:,18)),'k--','LineWidth',1.3,'DisplayName','共同β参考');xlim(ax,[0 8]);ylim(ax,[-33 3]);grid(ax,'on');xlabel(ax,'起漂后时间 [s]');ylabel(ax,'质心侧偏角 β [deg]');legend(ax,'Location','northeast');title(ax,{'2.5 s连续β调度：五种辅助侧倾刚度的实际β响应','同一工作点、3×5增益、参考与执行预算；无路径、外部ASB为0；未滤波。'});saveplot(f,out,'01_beta_five.png');
f=figure('Position',[20 20 1400 1050]);L=tiledlayout(3,2,'TileSpacing','compact','Padding','compact');title(L,'状态比较：左起漂起点对齐，右完整状态任务（绝对时间）');
for row=1:3,for side=1:2
 ax=nexttile(L);hold(ax,'on');for k=1:5,z=Z{k};p=z.repair_plant_raw;ix=1:20:size(p,1);q=p(ix,1);if side==1,q=q-z.start;end;y=[hypot(p(ix,3),p(ix,4))/3.6,p(ix,5),deg2rad(p(ix,6))];plot(ax,q,y(:,row),'Color',colors(k,:),'DisplayName',labels{k});end
 z=Z{1};q=z.repair_entry_diag(:,1);if side==1,q=q-z.start;end;r=z.repair_entry_diag(:,17:19);r(:,2)=rad2deg(r(:,2));plot(ax,q,r(:,row),'k--','DisplayName','基线参考');grid(ax,'on');if side==1,xlim(ax,[0 8]);xlabel(ax,'起漂后时间 [s]');else,xlim(ax,[0 35]);xlabel(ax,'时间 [s]');end
 names={'V [m/s]','β [deg]','车体 AVz [rad/s]'};ylabel(ax,names{row});if row==1,legend(ax,'Location','best');end
end,end
saveplot(f,out,'02_states.png');
f=figure('Position',[20 20 1450 1050]);L=tiledlayout(3,2,'TileSpacing','compact','Padding','compact');title(L,'请求、预算后目标与控制负担：起漂起点对齐');
for j=1:6
 ax=nexttile(L);hold(ax,'on');for k=1:5,z=Z{k};e=z.repair_entry_diag;a=z.repair_actuator_command;ix=1:20:size(a,1);q=e(:,1)-z.start;
  if j==1,y=rad2deg(e(:,27));elseif j==2,y=rad2deg(e(:,30));elseif j==3,y=e(:,29);elseif j==4,y=e(:,32);elseif j==5,q=a(ix,1)-z.start;y=max(abs(a(ix,6:9)),[],2);else,q=a(ix,1)-z.start;y=max(abs(a(ix,4:5)),[],2);end
  plot(ax,q,y,'Color',colors(k,:),'DisplayName',labels{k});
 end
 labs={'转向原始请求 [deg]','转向预算后目标 [deg]','后滑移原始请求','后滑移预算后目标','四轮矩绝对最大 [N·m]','实际转向速率绝对最大 [deg/s]'};ylabel(ax,labs{j});xlabel(ax,'起漂后时间 [s]');xlim(ax,[0 8]);grid(ax,'on');if j==1,legend(ax,'Location','best');end
end
saveplot(f,out,'03_inputs_constraints.png');
f=figure('Position',[20 20 1900 1200]);L=tiledlayout(4,4,'TileSpacing','compact','Padding','compact');title(L,'四轮轮胎状态：每条颜色对应同一刚度，起漂后0—8 s');
cols={28:31,10:13,14:17,18:21};names={'κ','Fx [N]','Fy [N]','Fz [N]'};wh={'FL','FR','RL','RR'};
for row=1:4,for wheel=1:4
 ax=nexttile(L);hold(ax,'on');for k=1:5,z=Z{k};p=z.repair_plant_raw;ix=1:20:size(p,1);plot(ax,p(ix,1)-z.start,p(ix,cols{row}(wheel)),'Color',colors(k,:),'DisplayName',labels{k});end
 xlim(ax,[0 8]);grid(ax,'on');title(ax,[wh{wheel} ' ' names{row}]);if row==4,xlabel(ax,'起漂后时间 [s]');end;if row==1&&wheel==1,legend(ax,'Location','best');end
end,end
saveplot(f,out,'04_tires_loads.png');
f=figure('Position',[20 20 1400 1100]);L=tiledlayout(3,2,'TileSpacing','compact','Padding','compact');title(L,'自然轨迹、辅助侧倾响应与后续阶段（轨迹不参加验收）');
for j=1:6
 ax=nexttile(L);hold(ax,'on');for k=1:5,z=Z{k};p=z.repair_plant_raw;s=z.asb_suspension;ix=1:20:size(p,1);
 if j==1,q=p(ix,36);y=p(ix,37);elseif j==2,q=s(ix,1)-z.start;y=s(ix,27);elseif j==3,q=p(ix,1)-z.hold;y=p(ix,5)+30;elseif j==4,q=p(ix,1)-z.exit;y=p(ix,5);elseif j==5,q=s(ix,1)-z.start;y=s(ix,3);else,q=s(ix,1)-z.start;y=s(ix,21);end
 plot(ax,q,y,'Color',colors(k,:),'DisplayName',labels{k});end
 grid(ax,'on');if j==1,axis(ax,'equal');xlabel(ax,'X [m]');ylabel(ax,'Y [m]');elseif j==3,xlim(ax,[0 10]);xlabel(ax,'实际维持后时间 [s]');ylabel(ax,'β误差 [deg]');elseif j==4,xlim(ax,[0 10]);xlabel(ax,'实际退漂后时间 [s]');ylabel(ax,'β [deg]');else,xlim(ax,[0 8]);xlabel(ax,'起漂后时间 [s]');if j==2,ylabel(ax,'后轴内部Aux力矩 [N·m]');elseif j==5,ylabel(ax,'后轴相对侧倾角 [deg]');else,ylabel(ax,'后右CmpS [mm]');end,end
 if j==1||j==2,legend(ax,'Location','best');end
end
saveplot(f,out,'05_trajectory_aux_hold_exit.png');
end
function saveplot(f,out,name),exportgraphics(f,fullfile(out,name),'Resolution',140);close(f);end
