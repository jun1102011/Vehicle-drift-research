%% a001_step37a_initialize.m：无路径目标状态漂移的唯一初始化入口
% runner先选择profile_id及aux_case_cfg；本入口每次从冻结JSON重建四套数值结构。
% 保持五状态反馈和车辆物理链；公共执行仅取消后轴目标滑移0.4上限，其他预算冻结。

% 不清空调用者工作区；本入口只初始化冻结状态银行，不启动仿真。
% profile缺失、形状/单位矛盾时停止，不自动求平衡点或反馈增益。
assert(exist('profile_id','var')==1 && (ischar(profile_id)||isstring(profile_id)),...
    'Step37a必须显式指定profile_id');
profile_id=char(profile_id);
assert(~isempty(regexp(profile_id,'^[A-Za-z][A-Za-z0-9_]*$','once')),'profile_id含非法路径字符');
step37a_root=fileparts(fileparts(mfilename('fullpath')));
profile_source=fullfile(step37a_root,'source','profiles',[profile_id '.json']);
assert(isfile(profile_source),'指定的Step37a工作点profile尚未生成');
profile_cfg=jsondecode(fileread(profile_source));
assert(isstruct(profile_cfg)&&isfield(profile_cfg,'profile_id')&&strcmp(profile_cfg.profile_id,profile_id),...
    'profile JSON身份与选择不符');
assert(all(isfield(profile_cfg,{'workpoint','entry_normal','entry_nodes','exit_nodes','state_protocol'})),'状态profile字段缺失');
wp=profile_cfg.workpoint;
wp_fields={'V_mps','beta_rad','AVz_rad_s','feedforward_3','offset_4','gain_3x5'};
assert(isstruct(wp)&&all(isfield(wp,wp_fields)),'profile工作点缺少V/β/AVz/前馈/偏置/3×5增益');
assert(isscalar(wp.V_mps)&&wp.V_mps>0&&isscalar(wp.beta_rad)&&...
    isscalar(wp.AVz_rad_s)&&all(isfinite([wp.V_mps,wp.beta_rad,wp.AVz_rad_s]))&&...
    isequal(size(wp.feedforward_3),[3,1])&&...
    isequal(size(wp.offset_4),[4,1])&&...
    isequal(size(wp.gain_3x5),[3,5])&&...
    all(isfinite(wp.feedforward_3(:)))&&all(isfinite(wp.offset_4(:)))&&...
    all(isfinite(wp.gain_3x5(:))),'工作点尺寸或有限性错误');
fid=fopen(profile_source,'rb');assert(fid>=0);profile_bytes=fread(fid,Inf,'*uint8');fclose(fid);
profile_md=java.security.MessageDigest.getInstance('SHA-256');
profile_md.update(typecast(profile_bytes,'int8'));
profile_source_sha256=lower(reshape(dec2hex(typecast(profile_md.digest(),'uint8'),2)',1,[]));

%% =========================
% 1) 车辆参数输入（来自当前CarSim配置及其Echo；不代表部件实测验证）
% =========================
m = 1843;     % 整车质量 [kg]
Iz = 3462.948099;     % 转动惯量 [kg*m^2]
a = 1.177744981;     % 前轴到质心距离 [m]
b = 1.682255019;     % 后轴到质心距离 [m]
% 当前CarSim轮胎表在静载、零侧偏/零纵滑附近的等效轴级刚度；不是全域Fiala等价。
Caf = 165585.267615143;   % 前轴侧偏刚度 [N/rad]
Car = 118001.807784225;    % 后轴侧偏刚度 [N/rad]
d = 1.675;      % 轮距 [m]
h = 0.4942892024;     % 质心高度 [m]
Rw = 0.32;     % 车轮半径 [m]
Iw = 1.5;      % 车轮转动惯量 [kg*m^2]
mu = 0.85;

% 论文方向盘等效约束除以14.6得到前轮约束；不再连接方向盘接口或齿条查表。

kw=5;       % 旧wheel_execution支路参数；state-only公共执行的轮速增益由entry_cfg.wheel_kw=12提供。

% 下列旧状态调节系数供路径支路/力诊断；state-only执行请求由profile的3×5 K给出。
k_beta_p = 1.5;
k_beta_d = 0.1;
k_beta = 0.1;
k_r_p = 5;
k_r_d = 0.50;
k_V = 2.0;

%% 原普通模式常量；state-only模式只保留未选中路径支路所需接口
% 车辆参数不由本入口反向覆盖CarSim。当前公共执行的幅值/变化率预算来自entry_cfg。

% 不使用附着、轮荷、转向幅值或力矩可行域门槛；保留连续数值正则化。

% 以下值是当前CarSim参数模型的追溯字段，不代表实物测量或范围认证。
carsim_cfg.M_SU = 1600;
carsim_cfg.LX_CG_SU = 1170;
carsim_cfg.H_CG_SU = 520;
carsim_cfg.IZZ_SU = 2800;
carsim_cfg.track_mm = 1675;
carsim_cfg.RRE_mm = 320;
carsim_cfg.IT = 1.5;
carsim_cfg.alpha_scale = [1,1];
% Step37a从已审核Step33d配置仅延长有效TSTOP；新研究par需独立逐case审核。
assert(exist('aux_case_cfg','var')==1 && isstruct(aux_case_cfg),'需先选择已审核的研究刚度配置');
carsim_cfg.source_path = aux_case_cfg.source_path;
carsim_cfg.source_sha256 = aux_case_cfg.source_sha256; % 本组实际派生par SHA，旧银行仅作接口来源
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
path_cfg.ramp_time=2;path_cfg.care_tol=1e-8; % 路径支路关闭；常量保留供图内类型/尺寸接口
path_cfg.mode=6;path_cfg.exit_s=NaN; % 6=普通开口轨迹；不循环、不启用漂移分支
path_Q=[10,0,0;0,10,0;0,0,2];path_R=[100,0,0;0,100,0;0,0,10]; % 冻结常量；本模式不调用路径LQR设计
addpath(fullfile(repair_root,'repair_support'));
% 入漂诊断：侧偏过渡长度和相对曲率的延后距离；均为工程工况参数，单位m。
path_cfg.beta_transition=path_cfg.transition;path_cfg.beta_delay=0;
path_geometry=repair_build_geometry(path_cfg); % 仅生成旧路径端口所需几何；不求根、不计算反馈增益
%% 原始控制接口：冻结物理/诊断参数，state-only执行增益来自profile
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

%% CarSim只引用已审核的本批派生文件
% 不调用repair_prepare_carsim；初始化只读sim/par，不创建或改写输入配置。
repair_simfile=aux_case_cfg.simfile_path;
assert(isfile(repair_simfile),'本批研究sim文件不存在，须先完成配置准备与审核。');
addpath('C:\Program Files (x86)\CarSim2020.0_Prog\Programs\solvers\Matlab84+');
fprintf('模型使用已审核研究配置：%s\n',repair_simfile);



%% 无路径状态任务：只加载预先冻结的工作点，不求解/整定/读取路线几何。
repair_run_mode='drift_cycle'; t_final=60;
entry_bank_source=jsondecode(fileread(fullfile(repair_root,'repair_support','repair_entry_bank.json')));
entry_cfg=rmfield(entry_bank_source,{'labels','source_hashes','note','source_par_sha256'});
assert(isequal(entry_cfg.slip_high(:),[.05;.4]),'源银行预算变化');
entry_cfg.slip_high(2)=Inf;rear_slip_upper_disabled=true;
entry_cfg.normal=profile_cfg.entry_normal;entry_cfg.nodes=profile_cfg.entry_nodes;
cycle_cfg=jsondecode(fileread(fullfile(repair_root,'repair_support','repair_cycle_bank.json')));
cycle_cfg.nodes=profile_cfg.exit_nodes;
state_cfg=profile_cfg.state_protocol;
state_cfg.target=state_cfg.target(:);state_cfg.right_target=state_cfg.right_target(:);state_cfg.entry_tolerance=state_cfg.entry_tolerance(:);
state_cfg.straight_tolerance=state_cfg.straight_tolerance(:);
assert(state_cfg.path_enabled==0&&state_cfg.entry_samples==600&&state_cfg.maintain_samples==20000&&state_cfg.straight_samples==2000);
assert(numel(entry_cfg.nodes)==9&&numel(cycle_cfg.nodes)==7);
repair_cfg.entry_mode=1;repair_cfg.cycle_mode=1;repair_cfg.stadium_mode=0;
repair_cfg.entry_steer_rate=entry_cfg.steering_rate_rad_s;path_cfg.feedback_enabled=0;
% 旧路径图仍保留接口，以下仅为未选中参数占位，没有路线或投影信息。
stadium_cfg=struct('disabled',1);
assert(abs(abs(wp.beta_rad)-pi/6)<1e-10&&abs(wp.V_mps-12)<1e-10);
fid=fopen(carsim_cfg.source_path,'rb');assert(fid>=0);bytes=fread(fid,Inf,'*uint8');fclose(fid);
md=java.security.MessageDigest.getInstance('SHA-256');md.update(typecast(bytes,'int8'));
digest=lower(reshape(dec2hex(typecast(md.digest(),'uint8'),2).',1,[]));
assert(strcmp(digest,carsim_cfg.source_sha256),'实际参数文件哈希不符');
fields=fieldnames(entry_cfg.vehicle);
for k=1:numel(fields),assert(abs(repair_cfg.(fields{k})-entry_cfg.vehicle.(fields{k}))<1e-8);end
fprintf('Step37a无路径：profile=%s；2.5s连续β参考、首个固定10s维持、本批指定退出调度；请求60s。\n',profile_id);
%% 理想主动稳定杆：五组固定辅助刚度；只在本研究副本生效
% 在运行入口前设置asb_case，baseline/front_soft_rear_hard/soft_soft/hard_hard/front_hard_rear_soft。
% Step33d：内部辅助刚度由GUI逐case改变；此处baseline仅表示外加ASB为零。
assert(strcmp(aux_case_cfg.schema,'step37a-state-only-file-case-v1'),'仅接受Step37a已审核文件派生身份');
if ~exist('asb_case','var'), asb_case='baseline'; end
asb_names={'baseline','front_soft_rear_hard','soft_soft','hard_hard','front_hard_rear_soft'};
asb_targets=[384 384;288 480;288 288;480 480;480 288];
asb_index=find(strcmp(asb_case,asb_names),1);assert(~isempty(asb_index),'未知主动稳定杆实验组');
asb_cfg=struct;asb_cfg.target_K_deg=asb_targets(asb_index,:)';
asb_cfg.delta_K_deg=asb_cfg.target_K_deg-[384;384];
asb_cfg.motion_ratio=[.959;.959;.797;.797]; % 历史CarSim Echo参数模型，非实物运动比验证
asb_cfg.track_m=[1.675;1.675]; % 实时由SusY修正实际力偶臂长
asb_cfg.Ts=Ts_plant;
asb_cfg.import_order=[1;3;2;4]; % GUI实际追加L1,L2,R1,R2；内部FL,FR,RL,RR
fprintf('外部附加ASB分支：%s；名义前/后标签 %.0f/%.0f N*m/deg（真实内部K以下方case为准）\n',asb_case,asb_cfg.target_K_deg);


%% 冻结五状态局部反馈：三个执行请求只使用前五项误差
% K为3×5，误差顺序[V(m/s), beta(rad), AVz(rad/s), kappa_f, kappa_r]。
% x仍为七项物理状态；Roll/AVx只用于状态记录和原接管门控，不参与K乘法。
% 所有工作点与空间增益已在JSON中直接冻结，不读取旧对照库或选择其他反馈。
feedback_mode='five';
assert(isequal(size(entry_cfg.normal.K),[3,5]),'普通点反馈矩阵必须为3×5');
for j=1:numel(entry_cfg.nodes),assert(isequal(size(entry_cfg.nodes(j).K),[3,5]),'起漂点反馈矩阵必须为3×5');end
for j=1:numel(cycle_cfg.nodes),assert(isequal(size(cycle_cfg.nodes(j).K),[3,5]),'退出点反馈矩阵必须为3×5');end

fprintf('局部反馈：V、beta、AVz、前轴滑移、后轴滑移；K为3×5。\n');

% Step33：起漂单轴开环脉冲，仅此研究副本启用
if ~exist('asb_pulse_cfg','var'),asb_pulse_cfg=struct('moment_Nm',[0;0],'start_s',0,'end_s',0,'edge_s',.1);end
assert(strcmp(asb_case,'baseline'),'Step33保持CarSim内部稳定杆基线');
assert(numel(asb_pulse_cfg.moment_Nm)==2 && all(isfinite(asb_pulse_cfg.moment_Nm)));
assert(asb_pulse_cfg.edge_s==.1 && (asb_pulse_cfg.end_s==asb_pulse_cfg.start_s || abs(asb_pulse_cfg.end_s-asb_pulse_cfg.start_s-.5)<1e-9));
asb_cfg.pulse_moment_Nm=asb_pulse_cfg.moment_Nm(:);
asb_cfg.pulse_start_s=asb_pulse_cfg.start_s;asb_cfg.pulse_end_s=asb_pulse_cfg.end_s;asb_cfg.pulse_edge_s=asb_pulse_cfg.edge_s;

%% Step33d固定配置对照：外加ASB始终零，真实内部K从GUI证据读取
assert(strcmp(asb_case,'baseline'),'固定内部刚度对照要求外加ASB baseline');
assert(all(asb_cfg.pulse_moment_Nm==0) && all(asb_cfg.delta_K_deg==0),'禁止叠加外加ASB干预');
fprintf('本组内部AuxRoll刚度：前/后%.0f/%.0f N*m/deg；外加四路零；仅后轴目标滑移上界按研究授权取消\n',aux_case_cfg.kf_deg,aux_case_cfg.kr_deg);
