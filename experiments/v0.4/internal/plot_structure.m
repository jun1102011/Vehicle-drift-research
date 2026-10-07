function plot_structure()
w=fileparts(fileparts(mfilename('fullpath')));out=fullfile(w,'results','analysis','figures');set(groot,'defaultFigureVisible','off','defaultTextFontName','Microsoft YaHei','defaultAxesFontName','Microsoft YaHei','defaultTextInterpreter','none');
f=figure('Position',[20 20 1450 650]);ax=axes(f,'Position',[0 0 1 1]);axis(ax,[0 1 0 1]);axis(ax,'off');
box(ax,[.02 .58 .24 .27],{'冻结连续参考 / 状态调度','预加载仍按旧公式至0.374413 s','其后β一次五次平滑曲线至−30°','同一β定位原节点，共享插值比例','x / 前馈 / K / 四轮偏置同步'},[1 .91 .8]);
box(ax,[.31 .58 .20 .27],{'五状态反馈','[V, β, AVz, κf, κr]','请求 = 前馈 − K×误差','K为3×5，节点数值冻结'},[.85 .93 1]);
box(ax,[.56 .58 .21 .27],{'原公共执行层','幅值与变化率预算','实际Fx补偿＋轮惯量','轮速预算＋±3000 N·m'},[.85 .93 1]);
box(ax,[.82 .58 .16 .27],{'CarSim 车辆','12输入 / 76输出','Aux：五种既定K','外部ASB四路为0'},[.85 .93 1]);
annotation(f,'arrow',[.26 .31],[.72 .72]);annotation(f,'arrow',[.51 .56],[.72 .72]);annotation(f,'arrow',[.77 .82],[.72 .72]);
box(ax,[.30 .12 .43 .22],{'原生状态与事件反馈','0.0005 s数据：600点起漂 / 2000点恢复','首次固定10 s维持窗口；不出带重计','XY、路径投影、方向误差均不参与控制'},[.89 .96 .89]);
annotation(f,'arrow',[.9 .9],[.58 .23]);annotation(f,'arrow',[.9 .73],[.23 .23]);annotation(f,'arrow',[.40 .40],[.34 .58]);annotation(f,'arrow',[.30 .14],[.23 .23]);annotation(f,'arrow',[.14 .14],[.23 .58]);
text(ax,.5,.95,'Step35g 实际启用结构：新增四组、引用一组，准备/维持/退漂及验收均冻结','HorizontalAlignment','center','FontSize',15);
exportgraphics(f,fullfile(out,'06_actual_structure.png'),'Resolution',150);saveas(f,fullfile(out,'06_actual_structure.svg'));close(f);
end
function box(ax,pos,lines,color)
rectangle(ax,'Position',pos,'FaceColor',color,'EdgeColor',[.25 .35 .4],'LineWidth',1.3);
text(ax,pos(1)+pos(3)/2,pos(2)+pos(4)/2,lines,'HorizontalAlignment','center','VerticalAlignment','middle','FontSize',11);
end
