%% a001_new_Design_LQR_Controller.m：第四章路径跟踪与分阶段内环维修
% V0.1/0.2/0.3统一五状态反馈；默认基准384/384，额外稳定杆输入为零。
% 唯一日常入口只初始化；drift_stadium不启用原路径LQR或Fiala反演执行支路。

% 不清空调用者工作区；本入口只初始化参数与几何表，不启动仿真。

%% =========================
% 1) 车辆参数输入（车辆沿用实测echo；接口以本交付审阅的12输入/76输出GUI配置为准）
% =========================
m = 1843;     % 整车质量 [kg]
Iz = 3462.948099;     % 转动惯量 [kg*m^2]
a = 1.177744981;     % 前轴到质心距离 [m]
b = 1.682255019;     % 后轴到质心距离 [m]
% 当前 CarSim 轮胎表在静载、零侧偏/零纵滑附近的等效轴级刚度；不是全域 Fiala 等价。
Caf = 165585.267615143;   % 前轴侧偏刚度 [N/rad]
Car = 118001.807784225;    % 后轴侧偏刚度 [N/rad]
d = 1.675;      % 轮距 [m]
h = 0.4942892024;     % 质心高度 [m]
Rw = 0.32;     % 车轮半径 [m]
Iw = 1.5;      % 车轮转动惯量 [kg*m^2]
mu = 0.85;

% 论文方向盘等效约束除以14.6得到前轮约束；不再连接方向盘接口或齿条查表。

kw=5;       %轮速反馈增益:实际轮速比目标轮速高 1 rad/s，kw=5 就会产生 −5 N·m 的转矩修正，用来减小轮速误差。

% 左右目标纵向力按CarSim四轮实际轮荷分配；不再使用固定前后载荷转移系数。
k_beta_p = 1.5;
k_beta_d = 0.1;
k_beta = 0.1;
k_r_p = 5;
k_r_d = 0.50;
k_V = 2.0;

% k_beta_p = 1.5;
% k_beta_d = 0.1;
% k_beta = 0.1;
% k_r_p = 5;
% k_r_d = 0.50;
% k_V = 2;

%% 原普通模式参数；局部起漂/退出模式的执行预算由工作点库集中定义
% 车辆参数和控制增益沿用上面的定义；执行输出不经过人为可行域门控。

% 不使用附着、轮荷、转向幅值或力矩可行域门槛；保留连续数值正则化。

% CarSim 是车辆参数来源；以下值用于追溯，不再反向覆盖 CarSim 车辆。
carsim_cfg.M_SU = 1600;
carsim_cfg.LX_CG_SU = 1170;
carsim_cfg.H_CG_SU = 520;
carsim_cfg.IZZ_SU = 2800;
carsim_cfg.track_mm = 1675;
carsim_cfg.RRE_mm = 320;
carsim_cfg.IT = 1.5;
carsim_cfg.alpha_scale = [1,1];
carsim_cfg.source_path = 'C:\Users\Public\Documents\CarSim2020.0_Data\Results\Run_54cb954f-d91a-4b4d-b546-9ec1dbf045e8\Run_all.par';
carsim_cfg.source_sha256 = '2b045ad9fd00a4f5f685b4ce6f603b1f464691a00beb194c792ae9deb2717fd6';
repair_root = fileparts(mfilename('fullpath'));
% 主链已采用直接前轮角，旧齿条查表仅留在维修快照中。
Ts_plant = 0.0005;             % CarSim交换与ode4固定积分步长 [s]

%% 分阶段工况：普通直线—左转—直线；不规划漂移动作
Ts=0.01; t_final=17; R=80; Vr=10.5; beta_r=0;
% 普通工况为工程诊断：名义横向加速度Vr^2/R=1.378m/s^2。
% 40m初始直线、25m入弯、40m恒曲率、25m出弯，然后保持直线。
% mode:1入漂维持；2同步退出；3先beta回正再降曲率；4原操场；5提前回正操场；6普通转弯。
% 独立退出的exit_s只由通过3秒稳态门槛的前置试验生成，不使用猜测初值。
path_cfg.R=R;path_cfg.straight=40;path_cfg.transition=25;
path_cfg.plateau=40; % 本普通工况恒曲率弧长[m]，总转角(25+40)/80=46.55deg
path_cfg.half=path_cfg.straight+2*path_cfg.transition+path_cfg.plateau;
path_cfg.length=2*path_cfg.half;path_cfg.V=Vr;path_cfg.beta=beta_r;
path_cfg.ramp_time=2;path_cfg.care_tol=1e-8; % CARE残差仅记录，不作为仿真门槛
path_cfg.mode=6;path_cfg.exit_s=NaN; % 6=普通开口轨迹；不循环、不启用漂移分支
path_Q=diag([10,10,2]);path_R=diag([100,100,10]);
addpath(fullfile(repair_root,'repair_support'));
% 入漂诊断：侧偏过渡长度和相对曲率的延后距离；均为工程工况参数，单位m。
path_cfg.beta_transition=path_cfg.transition;path_cfg.beta_delay=0;
path_geometry=repair_build_geometry(path_cfg);
%% 原始控制接口：物理参数与控制增益统一来自以上入口
repair_cfg=struct;
repair_cfg.m=m;
repair_cfg.Iz=Iz;
repair_cfg.a=a;
repair_cfg.b=b;
repair_cfg.d=d;
repair_cfg.h=h;
repair_cfg.Rw=Rw;
repair_cfg.Iw=Iw;
repair_cfg.mu=mu;
repair_cfg.Caf=Caf;
repair_cfg.Car=Car;
repair_cfg.kw=kw;
repair_cfg.Ts=Ts;
repair_cfg.Ts_plant=Ts_plant;
repair_cfg.k_beta_p=k_beta_p;
repair_cfg.k_beta_d=k_beta_d;
repair_cfg.k_beta=k_beta;
repair_cfg.k_r_p=k_r_p;
repair_cfg.k_r_d=k_r_d;
repair_cfg.k_V=k_V;
% 仅用于平滑轮速逆解在Fy过零处的数值变化，不限制目标力或执行器输出。
repair_cfg.force_eps_frac=0.02; % 按轴载荷容量比例设置连续正则化尺度
repair_interface_test=0;             % 仅验证脚本临时置1；日常闭环不旁路控制

%% CarSim只引用用户GUI导出的现有文件
% 不调用repair_prepare_carsim，不生成或改写任何sim/par。
repair_simfile='C:\Users\Public\Documents\CarSim2020.0_Data\Extensions\Simulink\00carsim_workspace\009\stabilizer_bar\simfile.sim';
assert(isfile(repair_simfile),'现有GUI sim文件不存在，请由用户在CarSim中重新生成。');
addpath('C:\Program Files (x86)\CarSim2020.0_Prog\Programs\solvers\Matlab84+');
fprintf('模型使用现有GUI配置：%s\n',repair_simfile);


%% 可选研究模式：普通直行→滑移建立→入漂→局部维持
% 在运行本入口前设置repair_run_mode='drift_entry'启用；'normal'保持原普通路径模式。
% 此入口只初始化，不启动仿真。25m/50m均通过原生对照，默认25m。
if ~exist('repair_run_mode','var'), repair_run_mode='drift_stadium'; end
assert(ismember(string(repair_run_mode),["normal","drift_entry","drift_cycle","drift_turn","drift_stadium"]),'repair_run_mode只支持normal、drift_entry、drift_cycle、drift_turn或drift_stadium');
if ~exist('repair_entry_length','var'), repair_entry_length=25; end
assert(ismember(repair_entry_length,[25,50]),'入漂长度仅有25m和50m经过本轮对照');
entry_bank_source=jsondecode(fileread(fullfile(repair_root,'repair_support','repair_entry_bank.json')));
% 参数只把数值结构送入MATLAB Function；来源、中文说明和标签留在工作区供核对。
entry_cfg=rmfield(entry_bank_source,{'labels','source_hashes','note','source_par_sha256'});
entry_cfg.length=repair_entry_length;
repair_cfg.entry_steer_rate=entry_cfg.steering_rate_rad_s;
repair_cfg.entry_mode=double(ismember(string(repair_run_mode),["drift_entry","drift_cycle","drift_turn","drift_stadium"]));
repair_cfg.cycle_mode=double(ismember(string(repair_run_mode),["drift_cycle","drift_turn","drift_stadium"]));
% 退出工作点与起漂库独立，数值库由当前车辆完整配平和自由释放校核。
cycle_cfg=jsondecode(fileread(fullfile(repair_root,'repair_support','repair_cycle_bank.json')));
path_cfg.feedback_enabled=1-repair_cfg.entry_mode;
if repair_cfg.entry_mode==1
    % 控制库与当前GUI配置绑定，避免更换车辆后仍套用这一组局部增益。
    fid=fopen(carsim_cfg.source_path,'rb');assert(fid>=0);bytes=fread(fid,Inf,'*uint8');fclose(fid);
    md=java.security.MessageDigest.getInstance('SHA-256');md.update(bytes);
    digest=lower(reshape(dec2hex(typecast(md.digest(),'uint8'),2).',1,[]));
    % GUI每次导出会更新File created注释；这条时间说明不参与车辆计算。
    % 保留完整哈希优先检查；仅对已验收库允许忽略该行文字，其他字节仍严格核对。
    if ~strcmp(digest,entry_bank_source.source_par_sha256)
        par_text=char(bytes.'); % 一字节对应一字符，避免改变原文件编码或换行。
        par_stable=regexprep(par_text,'(?m)^! File created:[^\r\n]*','');
        md_stable=java.security.MessageDigest.getInstance('SHA-256');
        md_stable.update(uint8(par_stable));
        stable_digest=lower(reshape(dec2hex(typecast(md_stable.digest(),'uint8'),2).',1,[]));
        % 来源：已通过正式40s操场试验的LastRun_all.par；只剔除上述时间注释。
        validated_raw='9ec11188dd38f8b1d1372843de1354e71b92b0f91f22e877e2ded59abb6908bd';
        validated_stable='35ca7a42b0e4ab786929c275ce9d26eea151da55f0acfa5e020d0c77b959615f';
        same_validated_config=strcmp(entry_bank_source.source_par_sha256,validated_raw) && strcmp(stable_digest,validated_stable);
        if ~same_validated_config, asb_validate_config(carsim_cfg.source_path); end
        fprintf('研究配置校验通过：使用已审阅的12/76接口；旧工作点性能需本轮回归。\n');
    end
    fields=fieldnames(entry_cfg.vehicle);
    for k=1:numel(fields),assert(abs(repair_cfg.(fields{k})-entry_cfg.vehicle.(fields{k}))<1e-8,'物理参数与工作点库不一致');end
    t_final=30;
    if ~ismember(string(repair_run_mode),["drift_turn","drift_stadium"])
    fprintf('有效运行模式：直行→入漂→维持，过渡%d m，目标12m/s、beta=-10deg、AVz=0.6rad/s；路径反馈关闭。\n',repair_entry_length);
    end
else
    fprintf('有效运行模式：原普通路径跟踪，17s、R=80m、beta_ref=0；新增局部漂移支路不驱动车辆。\n');
end

%% 可选完整起退漂：沿自然方向退出，不启用位置/路径反馈
% drift_entry保持v0.1起漂维持；drift_cycle新增B方案50m卸载/解除转弯。
% 此入口仍只初始化；日志repair_cycle_diag记录阶段0~6、门控和参考变化率。
if repair_cfg.cycle_mode==1 && ~ismember(string(repair_run_mode),["drift_turn","drift_stadium"])
    t_final=40;
    entry_cfg.length=25; % 已通过起漂长度冻结，退出长度来自cycle_cfg.length。
    fprintf('有效完整工况：起漂→维持至少10s→B方案退出50m→12m/s自然直行；请求40s。\n');
end

%% 研究单弯：已通过原生180度校核，独立Simulink复测；不是操场模式
% 不修改drift_cycle既有25m起漂、10s维持、50m退出参数。
if strcmp(repair_run_mode,'drift_turn')
    t_final=25;entry_cfg.length=20;
    cycle_cfg.nodes=cycle_cfg.nodes([1,4,5]);cycle_cfg.grid=[0;.5;1];
    cycle_cfg.scheme=3;cycle_cfg.length=35;cycle_cfg.hold_min_s=1.84;
    fprintf('研究单弯：起漂20m、稳定维持1.84s、重叠退出35m；无路径反馈，不是操场验收。\n');
end

%% 固定操场：研究版本，控制修正与车辆输入分层记录
stadium_cfg=jsondecode(fileread(fullfile(repair_root,'repair_support','repair_stadium_reference.json')));
repair_cfg.stadium_mode=double(strcmp(repair_run_mode,'drift_stadium'));
if repair_cfg.stadium_mode==1
    t_final=40;
    fprintf('固定操场：80m引入段＋两条60m直线＋两处20m稳定漂移弯；路径纠偏±0.03rad/s；请求40s。\n');
end

%% 理想主动稳定杆：五组固定辅助刚度；只在本研究副本生效
% 在运行入口前设置asb_case，baseline/front_soft_rear_hard/soft_soft/hard_hard/front_hard_rear_soft。
% 不是整轴总刚度，不是实测执行器参数；原CarSim内部384/384保持不变。
asb_validate_config(carsim_cfg.source_path);
if ~exist('asb_case','var'), asb_case='baseline'; end
asb_names={'baseline','front_soft_rear_hard','soft_soft','hard_hard','front_hard_rear_soft'};
asb_targets=[384 384;288 480;288 288;480 480;480 288];
asb_index=find(strcmp(asb_case,asb_names),1);assert(~isempty(asb_index),'未知主动稳定杆实验组');
asb_cfg=struct;asb_cfg.target_K_deg=asb_targets(asb_index,:)';
asb_cfg.delta_K_deg=asb_cfg.target_K_deg-[384;384];
asb_cfg.motion_ratio=[.959;.959;.797;.797]; % 当前echo：dCmp_Spring/dJounce
asb_cfg.track_m=[1.675;1.675]; % 实时由SusY修正实际力偶臂长
asb_cfg.Ts=Ts_plant;
asb_cfg.import_order=[1;3;2;4]; % GUI实际追加L1,L2,R1,R2；内部FL,FR,RL,RR
fprintf('理想主动稳定杆组：%s；前/后辅助刚度 %.0f/%.0f N*m/deg\n',asb_case,asb_cfg.target_K_deg);


%% 冻结五状态局部反馈：三个执行请求只使用前五项误差
% K为3×5，误差顺序[V(m/s), beta(rad), AVz(rad/s), kappa_f, kappa_r]。
% x仍为七项物理状态；Roll/AVx只用于状态记录和原接管门控，不参与K乘法。
% 所有工作点与空间增益已在JSON中直接冻结，不读取旧对照库或选择其他反馈。
feedback_mode='five';
assert(isequal(size(entry_cfg.normal.K),[3,5]),'普通点反馈矩阵必须为3×5');
for j=1:numel(entry_cfg.nodes),assert(isequal(size(entry_cfg.nodes(j).K),[3,5]),'起漂点反馈矩阵必须为3×5');end
for j=1:numel(cycle_cfg.nodes),assert(isequal(size(cycle_cfg.nodes(j).K),[3,5]),'退出点反馈矩阵必须为3×5');end
assert(size(stadium_cfg.K,2)==3 && size(stadium_cfg.K,3)==5,'空间反馈矩阵必须为N×3×5');
fprintf('局部反馈：V、beta、AVz、前轴滑移、后轴滑移；K为3×5。\n');
