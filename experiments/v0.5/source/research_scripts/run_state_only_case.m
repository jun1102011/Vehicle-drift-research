function run_state_only_case(profile_id,case_id)
%RUN_STATE_ONLY_CASE 按冻结β30状态协议和内部Aux刚度执行一组60s车辆仿真。
% profile_id仅允许既有R3；首组接口和配置审核后放行其他四刚度，性能失败保留。
% 本函数只读研究模型和已准备的独立sim/par，写独立结果目录与唯一56路无损MAT。
% 配置/源文件/控制参数错误立即停止；车辆或性能失败保留原始记录与CarSim证据。
w=fileparts(fileparts(mfilename('fullpath')));md=fullfile(w,'model');
step35a=fullfile(fileparts(w),'step35a_beta30_20261006');
oldpwd=pwd;restorepwd=onCleanup(@()cd(oldpwd));
oldfig=get(groot,'defaultFigureVisible');restorefig=onCleanup(@()set(groot,'defaultFigureVisible',oldfig));
set(groot,'defaultFigureVisible','off');
profile_id=char(profile_id);case_id=char(case_id);
assert(~isempty(regexp(profile_id,'^[A-Za-z][A-Za-z0-9_]*$','once')),'profile_id含非法路径字符');
assert(any(strcmp(profile_id,{'LeftRight_R3'})),...
 'Step37a仅许可冻结R3');
order={'kf384_kr384','kf192_kr384','kf576_kr384','kf384_kr192','kf384_kr576'};
assert(any(strcmp(case_id,order)),'本轮仅运行五种批准刚度，冻结同步连续左右换向调度');
profile_path=fullfile(w,'source','profiles',[profile_id '.json']);
assert(isfile(profile_path),'工作点profile缺失');profile=jsondecode(fileread(profile_path));
assert(isfield(profile,'profile_id')&&strcmp(profile.profile_id,profile_id),'profile身份不一致');
profile_sha=sha(profile_path);
cfg_path=fullfile(w,'source','file_cases',case_id,'aux_case_cfg.json');
assert(isfile(cfg_path),'Step37a独立CarSim配置尚未准备');aux=jsondecode(fileread(cfg_path));
pre=check_file_config(aux,case_id);source_freeze=checkfreeze(w);
gate=checkgate(w,profile_id,profile_sha,case_id,source_freeze);
run_id=[profile_id '_' case_id];folder=fullfile(w,'results','runs',run_id);
assert(~isfolder(folder),'本组运行记录已存在，拒绝覆盖');
model='stabilizer_bar_step37a';assert(~bdIsLoaded(model),'同名模型已加载，请先保存并关闭');
mkdir(folder);writejson(fullfile(folder,'case.json'),struct('profile_id',profile_id,'case_id',case_id,'duration_s',60,...
 'internal_K_Nm_per_deg',[aux.kf_deg aux.kr_deg],'external_ASB_Nm',[0 0],...
 'rear_slip_upper_disabled',true,'rear_slip_upper_value','+Inf',...
 'state_only_no_path',true,'first_frozen_maintain_s',10));
writejson(fullfile(folder,'source_identity.json'),struct('profile_path',profile_path,'profile_sha256',profile_sha,...
 'configuration_path',cfg_path,'configuration_sha256',sha(cfg_path),...
 'source_freeze_path',fullfile(w,'source','runner_freeze.json'),'source_freeze_sha256',source_freeze,...
 'gate',gate,'pre_run_config',pre));
sim_attempted=false;evidence_saved=false;archive_saved=false;sim_start_datenum=NaN;
try
 cd(md);addpath(fullfile(w,'internal'));addpath(fullfile(step35a,'internal'));
 % 入口变量从当前case/profile显式设置；InitFcn重入必须读同一来源。
 assignin('base','aux_case_cfg',aux);assignin('base','profile_id',profile_id);
 assignin('base','asb_case','baseline');assignin('base','repair_run_mode','drift_cycle');
 assignin('base','repair_entry_length',25);
 pulse=struct('moment_Nm',[0;0],'start_s',0,'end_s',0,'edge_s',.1);
 assignin('base','asb_pulse_cfg',pulse);
 init=fullfile(md,'a001_step37a_initialize.m');
 evalin('base',['run(''' strrep(init,'''','''''') ''');']);
 first=checkcontrol(aux,profile_id,profile_sha,'after_explicit_initializer');
 load_system(fullfile(md,[model '.slx']));
 assert(samepath(get_param(model,'FileName'),fullfile(md,[model '.slx'])),'实际加载模型不是研究副本');
 set_param(model,'SimulationCommand','update');
 second=checkcontrol(aux,profile_id,profile_sha,'after_model_update_InitFcn');
 assert(isequaln(first.actual,second.actual),'显式初始化与编译InitFcn的控制数值不同');
 assert(strcmp(get_param(model,'SolverType'),'Fixed-step')&&strcmp(get_param(model,'Solver'),'ode4'));
 solver_step=evalin('base',get_param(model,'FixedStep'));
 assert(isscalar(solver_step)&&abs(solver_step-.0005)<1e-12,'CarSim交换步长变化');
 assert(strcmp(get_param(model,'FastRestart'),'off')&&strcmp(get_param(model,'LoadInitialState'),'off'),...
  '不能沿用上次状态或FastRestart');
 % 标准JSON把+Inf写作null；独立保存一次原生double数值结构以完整保留研究上界。
 controller_numeric=first.actual;numeric_path=fullfile(folder,'controller_numeric.mat');
 save(numeric_path,'controller_numeric','-v7');
 numeric_readback=load(numeric_path,'controller_numeric');
 assert(isequaln(numeric_readback.controller_numeric,controller_numeric),...
  '控制数值结构写后读回不一致');
 writejson(fullfile(folder,'used_config.json'),struct('profile',profile,'profile_sha256',profile_sha,...
  'vehicle_config',aux,'controller',first.actual,'control_check_pre',rmfield(first,'actual'),...
  'control_check_update',rmfield(second,'actual'),'ordinary_initial_state',true,...
  'state_only_no_path',true,'path_feedback_enabled',false,...
  'rear_slip_upper_disabled',true,'rear_slip_upper_value','+Inf',...
  'controller_numeric_path',numeric_path,'controller_numeric_sha256',sha(numeric_path)));
 before_sim=check_file_config(aux,case_id);assert(isequaln(pre,before_sim),'准备阶段CarSim配置变化');
 assert(strcmp(source_freeze,checkfreeze(w))&&strcmp(profile_sha,sha(profile_path)),'仿真前源文件变化');
 in=Simulink.SimulationInput(model);in=in.setModelParameter('StopTime','60','CaptureErrors','on');
 sim_start_datenum=now;sim_attempted=true;
 out=sim(in); % 唯一实际启动60s车辆的语句；准备本文件时不调用。
 actual_end=0;if ismember('tout',out.who)&&~isempty(out.tout),actual_end=out.tout(end);end
 vehicle_error=char(out.ErrorMessage);
 writejson(fullfile(folder,'summary.json'),struct('run_id',run_id,'actual_end',actual_end,'error',vehicle_error,...
  'duration_s',60,'feedback_state_count',5,'internal_K_Nm_per_deg',[aux.kf_deg aux.kr_deg],...
  'ordinary_initial_state',true,'external_ASB_zero_required',true,...
  'task','state_only_no_path','performance_acceptance','pending_independent_analysis'));
 rawfile=fullfile(folder,'raw_numeric.mat');
 try
  meta=save_numeric_archive(out,rawfile);archive_saved=true;
 catch archive_error
  save(fullfile(folder,'raw_conversion_failure.mat'),'out','-v7.3');
  write_text(fullfile(folder,'archive_failure.txt'),getReport(archive_error,'extended','hyperlinks','off'));
  rethrow(archive_error);
 end
 evidence=copyevidence(pre.outputs,folder,aux,sim_start_datenum);evidence_saved=true;
 writejson(fullfile(folder,'CarSim_evidence_index.json'),evidence);
 verification=verify_archive(out,rawfile,meta);
 assert(numel(fieldnames(meta.channels))==56,'日志不再为56路；不得静默丢弃');
 writejson(fullfile(folder,'archive_index.json'),struct('archive_sha256',sha(rawfile),...
  'channel_count',56,'bitwise_readback',verification.all_equal,'channels',meta.channels,'checks',verification.checks));
 required={'repair_plant_raw','repair_actuator_command','repair_entry_diag','repair_cycle_diag',...
  'repair_stadium_diag','repair_steer_diag','asb_import12','asb_diag','asb_suspension','asb_roll_feedback','feedback_diag'};
 arrays=struct;for k=1:numel(required),assert(isfield(meta.channels,required{k}),'原生日志缺失:%s',required{k});...
  arrays.(required{k})=meta.channels.(required{k}).size;end
 writejson(fullfile(folder,'arrays.json'),arrays);
 inputs=checkinputs(rawfile,aux,folder);writejson(fullfile(folder,'input_verification.json'),inputs);
 post=check_file_config(aux,case_id);assert(isequaln(pre,post),'运行中CarSim配置变化');
 assert(strcmp(sha(pre.outputs.inputarchive),aux.source_sha256),'真实展开输入与本组配置不一致');
 third=checkcontrol(aux,profile_id,profile_sha,'after_sim_InitFcn');
 assert(isequaln(first.actual,third.actual),'仿真后InitFcn改变控制数值');
 assert(strcmp(source_freeze,checkfreeze(w))&&strcmp(profile_sha,sha(profile_path)),'仿真期间源文件变化');
 writejson(fullfile(folder,'post_run_check.json'),struct('config',post,'controller',third.actual,...
  'source_freeze_sha256',source_freeze,'profile_sha256',profile_sha));
 [ok,msg]=fileattrib(rawfile,'-w');assert(ok,'原始记录只读设置失败:%s',msg);
 close_system(model,0);
 vehicle_completed=isempty(vehicle_error)&&actual_end>=60-1e-8;
 state='vehicle_failure_archived';if vehicle_completed,state='completed_pending_performance_analysis';end
 writejson(fullfile(folder,'completion.json'),struct('state',state,'vehicle_completed',vehicle_completed,...
  'actual_end',actual_end,'archive_verified',true,'interface_verified',true,...
  'source_freeze_verified',true,'external_asb_zero',inputs.external_imports_exact_numeric_zero,...
  'performance_pending',true));
 fprintf('STEP37C_ARCHIVED %s end=%.6f vehicle_completed=%d\n',run_id,actual_end,vehicle_completed);
catch err
 % 已启动sim后才救援CarSim证据，避免把旧LastRun伪装为当前运行。
 if sim_attempted && ~evidence_saved
  try
   evidence=copyevidence(pre.outputs,folder,aux,sim_start_datenum);writejson(fullfile(folder,'CarSim_evidence_index.json'),evidence);
  catch evidence_error
   write_text(fullfile(folder,'evidence_copy_failure.txt'),getReport(evidence_error,'extended','hyperlinks','off'));
  end
 end
 write_text(fullfile(folder,'failure.txt'),getReport(err,'extended','hyperlinks','off'));
 writejson(fullfile(folder,'failure_status.json'),struct('run_id',run_id,'simulation_attempted',sim_attempted,...
  'numeric_archive_written',archive_saved,'exception',err.message,'records_preserved',true));
 if bdIsLoaded(model),close_system(model,0);end
 rethrow(err);
end
end

function proof=checkgate(w,id,profile_sha,case_id,source_sha)
% 只许可冻结连续换向R3；新零回归技术与性能通过后放行其余配置。
assert(strcmp(id,'LeftRight_R3'));
order={'kf384_kr384','kf192_kr384','kf576_kr384','kf384_kr192','kf384_kr576'};j=find(strcmp(order,case_id));assert(numel(j)==1);
if j>1
 g=jsondecode(fileread(fullfile(w,'audit','review_kf384_kr384.json')));
 assert(g.technical_pass&&g.performance_pass&&numel(g.full_regression_channels)==56,'384/384全程回归未通过');
 for k=1:j-1
  prior=jsondecode(fileread(fullfile(w,'audit',['review_' order{k} '.json'])));
  assert(prior.technical_pass,'上一组技术异常未解决；性能失败不阻止下一组');
 end
end
proof=struct('scope','Step37c uniform-plane straight road coordinates; frozen left-right controller; no ASB','source_freeze',source_sha,'max_full_runs',5);
end

function p=check_file_config(c,case_id)
% 父60s物理par逐字节一致；本批sim仅换INPUT和OUTPUT两个ASCII路径。
assert(strcmp(c.schema,'step37a-state-only-file-case-v1')&&strcmp(c.case,case_id));
work_root=fileparts(fileparts(mfilename('fullpath')));
manifest_path=fullfile(work_root,'source','config_manifest.json');
assert(isfile(manifest_path),'缺少本批配置派生审核清单');
manifest=jsondecode(fileread(manifest_path));
assert(strcmp(manifest.schema,'step37c-road-coordinate-config-manifest-v1')&&~manifest.gui_database_mutated);
matches=find(strcmp({manifest.cases.case},case_id));assert(numel(matches)==1,'manifest工况缺失或重复');
item=manifest.cases(matches);
cfg_path=fullfile(work_root,'source','file_cases',case_id,'aux_case_cfg.json');
assert(strcmp(item.configuration_identity_sha256,sha(cfg_path)),'manifest工况身份哈希变化');
assert(strcmp(item.derived_par_sha256,c.source_sha256)&&strcmp(item.derived_sim_sha256,c.simfile_sha256),...
 'manifest与实际派生配置身份不一致');
assert(c.effective_tstop_s==60&&c.opt_stop==0&&c.step_s==.0005&&...
 c.ports_import==12&&c.ports_export==76&&c.external_asb_zero_required);
assert(~c.gui_database_mutated&&strcmp(c.configuration_mode,'file_derived_road_coordinates_straight_uniform_plane'));
files={c.source_path,c.simfile_path,c.origin_cfg_path,c.origin_source_path,c.origin_simfile_path};
for k=1:numel(files),assert(isfile(files{k}),'CarSim配置文件缺失:%s',files{k});end
assert(strcmp(sha(c.source_path),c.source_sha256)&&strcmp(sha(c.simfile_path),c.simfile_sha256));
assert(strcmp(sha(c.origin_cfg_path),c.origin_cfg_sha256)&&...
 strcmp(sha(c.origin_source_path),c.origin_source_sha256)&&...
 strcmp(sha(c.origin_simfile_path),c.origin_simfile_sha256),'Step35b父配置来源哈希变化');
parent_cfg=jsondecode(fileread(c.origin_cfg_path));
assert(strcmp(parent_cfg.schema,'step35b-file-case-v1')&&strcmp(parent_cfg.case,case_id)&&...
 strcmp(parent_cfg.source_sha256,c.origin_source_sha256)&&...
 strcmp(parent_cfg.simfile_sha256,c.origin_simfile_sha256),...
 '父配置身份不等于Step35b已审核60s组');
base=readbytes(c.origin_source_path);actual=readbytes(c.source_path);
opts=regexp(base,'(?m)^OPT_STOP[ \t]+([^\r\n]+)','tokens');
assert(~isempty(opts)&&strcmp(strtrim(opts{end}{1}),'0'),'源文件最终OPT_STOP改变');
stop_hits=regexp(base,'(?m)^TSTOP[ \t]+([^\r\n]+)','tokens');
assert(~isempty(stop_hits)&&strcmp(strtrim(stop_hits{end}{1}),'60'),...
 'Step35b父配置有效TSTOP不是60');
% 本轮唯一变化是道路坐标；不改物理高度/附着/车辆/初态。
expected=replace_line(base,'OPT_PATH_LOOP','OPT_PATH_LOOP 0');
expected=replace_line(expected,'SEGMENT_TYPE','SEGMENT_TYPE 0');
expected=regexprep(expected,'SEGMENT_RADIUS 15\.000000\r?\nSEGMENT_ARC 360\.000000','SEGMENT_LENGTH 2000');
assert(strcmp(actual,expected),'道路坐标白名单以外字节变化');
runtime=fullfile('C:/Users/28583/AppData/Local/Temp/step37c_road_coordinate_20261010',case_id);
assert(samepath(c.source_path,fullfile(runtime,'Run_all.par'))&&...
 samepath(c.simfile_path,fullfile(runtime,'simfile.sim')),'运行路径不在独立ASCII目录');
sim=readbytes(c.origin_simfile_path);
expected_sim=replace_line(sim,'INPUT',['INPUT ' char(java.io.File(c.source_path).getCanonicalPath())]);
expected_sim=replace_line(expected_sim,'SET_MACRO \$\(OUTPUT_FILE_PREFIX\)\$',...
 ['SET_MACRO $(OUTPUT_FILE_PREFIX)$ ' char(java.io.File(fullfile(runtime,'LastRun')).getCanonicalPath())]);
assert(strcmp(readbytes(c.simfile_path),expected_sim),'sim的INPUT/OUTPUT以外字节变化');
q=resolve_sim(c.simfile_path);
assert(samepath(q.input,c.source_path)&&abs(str2double(q.step)-.0005)<1e-12);
assert(strcmp(strrep(q.imports,' ',''),'1,12')&&strcmp(strrep(q.exports,' ',''),'1,76'));
p=struct('case',case_id,'source_sha256',c.source_sha256,'simfile_sha256',c.simfile_sha256,...
 'origin_source_sha256',c.origin_source_sha256,'origin_simfile_sha256',c.origin_simfile_sha256,...
 'effective_tstop_s',60,'opt_stop',0,'byte_whitelist_passed',true,'outputs',q.outputs);
end

function text=readbytes(path)
f=fopen(path,'rb');assert(f>=0);clean=onCleanup(@()fclose(f));text=char(fread(f,Inf,'*uint8').');
end
function text=replace_line(text,key,replacement)
[a,b]=regexp(text,['(?m)^' key ' [^\r\n]*'],'start','end');assert(numel(a)==1);
text=[text(1:a-1) replacement text(b+1:end)];
end
function q=resolve_sim(path)
text=fileread(path);macros=regexp(text,'(?m)^SET_MACRO\s+(\S+)\s+([^\r\n]+)','tokens');
q=struct('input',getcommand('INPUT'),'step',getcommand('EXT_MODEL_STEP'),...
 'imports',getcommand('PORTS_IMP'),'exports',getcommand('PORTS_EXP'));
q.input=absolute(q.input);q.outputs=struct('inputarchive',absolute(getcommand('INPUTARCHIVE')),...
 'echo',absolute(getcommand('ECHO')),'final',absolute(getcommand('FINAL')),...
 'log',absolute(getcommand('LOGFILE')),'vs',absolute(getcommand('ERDFILE')));
[parent,stem]=fileparts(q.outputs.vs);q.outputs.vsb=fullfile(parent,[stem '.vsb']);
function value=getcommand(key)
 a=regexp(text,['(?m)^' key '\s+([^\r\n]+)'],'tokens');assert(numel(a)==1,'sim命令缺失/重复:%s',key);value=strtrim(a{1}{1});
 for pass=1:30
  previous=value;for j=1:numel(macros),value=strrep(value,macros{j}{1},strtrim(macros{j}{2}));end
  if strcmp(previous,value),break;end
 end
 assert(isempty(regexp(value,'\$\([^)]+\)\$','once')),'sim宏无法解析');
end
function value=absolute(value)
 f=java.io.File(value);if ~f.isAbsolute(),value=fullfile(fileparts(path),value);end
 value=char(java.io.File(value).getCanonicalPath());
end
end

function proof=checkcontrol(aux,id,profile_sha,stage)
% 三次核对实际状态协议、三套控制数值、无路径身份与公共执行预算。
actual=struct('entry',evalin('base','entry_cfg'),'cycle',evalin('base','cycle_cfg'),...
 'repair',evalin('base','repair_cfg'),'state',evalin('base','state_cfg'),...
 'path',evalin('base','path_cfg'));
baseline_path=fullfile(fileparts(fileparts(mfilename('fullpath'))),...
 '..','step33_asb_entry_map_20261004','source','common_used_config.json');
baseline_sha='a7ea710b260d8d36babe3f2ebb85538567b88224ab794b3b575f99cdae5a2255';
assert(strcmp(sha(baseline_path),baseline_sha),'原公共执行配置字节改变，拒绝继续');
baseline=jsondecode(fileread(baseline_path));
expected_repair=baseline.repair;expected_repair.stadium_mode=0;
assert(isequaln(actual.repair,expected_repair)&&actual.repair.entry_mode==1&&...
 actual.repair.cycle_mode==1&&actual.repair.stadium_mode==0,...
 'repair执行参数不是原冻结数值加授权的无路径模式');
% profile仅指定normal/entry/exit节点，公共执行只例外取消后轴κ幅值上界。
selected=evalin('base','profile_cfg');expected_entry=baseline.entry;
assert(isfield(selected,'workpoint')&&isfield(selected,'entry_normal')&&...
 isfield(selected,'entry_nodes')&&isfield(selected,'exit_nodes')&&...
 isfield(selected,'state_protocol'),'profile缺少Step37a状态接口');
expected_entry.normal=selected.entry_normal;
expected_entry.nodes=selected.entry_nodes;
assert(isequal(expected_entry.slip_high(:),[.05;.4]),'原前后滑移上界与冻结源不符');
expected_entry.slip_high(2)=Inf;
assert(evalin('base','rear_slip_upper_disabled')==true&&...
 isinf(actual.entry.slip_high(2))&&actual.entry.slip_high(2)>0,...
 'Step37a继承用户取消后轴目标滑移上界的研究身份未生效');
assert(isequaln(actual.entry,expected_entry),...
 'entry预算或非授权节点改变；只许可profile的normal/nodes与后轴上界+Inf');
expected_cycle=baseline.cycle;expected_cycle.nodes=selected.exit_nodes;
assert(isequaln(actual.cycle,expected_cycle)&&actual.cycle.hold_min_s==10,...
 '退出银行、维持时长或cycle公共参数改变');
expected_state=selected.state_protocol;
expected_state.target=expected_state.target(:);expected_state.right_target=expected_state.right_target(:);
expected_state.entry_tolerance=expected_state.entry_tolerance(:);
expected_state.straight_tolerance=expected_state.straight_tolerance(:);
assert(isequaln(actual.state,expected_state),...
 '运行中的状态协议不等于profile冻结内容');
assert(actual.state.entry_samples==600&&actual.state.maintain_samples==20000&&...
 actual.state.straight_samples==2000&&actual.state.native_step_s==.0005&&...
 actual.state.maintain_s==10&&actual.state.path_enabled==0,...
 '状态协议样本数/周期/路径开关改变');
assert(actual.path.feedback_enabled==0,'无路径状态任务仍启用了路径反馈');
wp=selected.workpoint;
assert(abs(wp.V_mps-12)<1e-10&&abs(abs(wp.beta_rad)-pi/6)<1e-10&&...
 wp.AVz_rad_s*wp.beta_rad<0,...
 '工作点不是已验V12 β−30 body AVz解');
assert(all(abs(actual.entry.nodes(end).x(1:3)-...
 [wp.V_mps;wp.beta_rad;wp.AVz_rad_s])<1e-9),...
 '漂移终点节点与本组已验工作点三状态不符');
assert(strcmp(evalin('base','profile_id'),id),'InitFcn改选了工作点');
assert(strcmp(evalin('base','profile_source_sha256'),profile_sha),'initializer未载入本组profile字节');
assert(strcmp(evalin('base','repair_run_mode'),'drift_cycle'),'进入了路径/操场控制模式');
assert(evalin('base','Ts')==.01&&evalin('base','Ts_plant')==.0005);
assert(strcmp(evalin('base','feedback_mode'),'five')&&strcmp(evalin('base','asb_case'),'baseline'));
assert(samepath(evalin('base','repair_simfile'),aux.simfile_path),'initializer未指向本组sim');
cs=evalin('base','carsim_cfg');assert(samepath(cs.source_path,aux.source_path)&&...
 strcmp(cs.source_sha256,aux.source_sha256),'initializer展开par身份不同');
asb=evalin('base','asb_cfg');pulse=evalin('base','asb_pulse_cfg');
assert(all(asb.delta_K_deg==0)&&all(asb.pulse_moment_Nm==0)&&all(pulse.moment_Nm==0));
assert(isequaln(evalin('base','aux_case_cfg'),aux),'InitFcn覆盖了CarSim来源');
proof=struct('stage',stage,'actual',actual,'profile_sha256',profile_sha,...
 'five_state_feedback',true,'external_ASB_zero_configuration',true,'Ts_s',.01,'Ts_plant_s',.0005,...
 'common_source_sha256',baseline_sha,'rear_slip_upper_disabled',true,...
 'rear_slip_upper_value','+Inf','path_feedback_disabled',true,...
 'state_protocol_equal_profile',true,'other_executor_fields_equal_frozen',true);
end

function evidence=copyevidence(paths,folder,c,started)
% 仅复制本次CarSim输出，避免R3/R1共用C盘前缀时把上组文件误认为当前证据。
keys={'echo','final','log','vs','vsb'};names={'LastRun_echo.par','LastRun_end.par','LastRun_log.txt','LastRun.vs','LastRun.vsb'};
evidence=struct('configuration_reference',c.origin_source_path,'configuration_sha256',c.source_sha256,'outputs',struct);
for k=1:numel(keys)
 source=paths.(keys{k});assert(isfile(source),'CarSim必要输出缺失:%s',source);
 info=dir(source);assert(info.datenum>=started-2/86400,...
  'CarSim输出早于本次仿真，拒绝复制旧证据:%s',source);
 target=fullfile(folder,[names{k} '.evidence']);assert(~isfile(target),'拒绝覆盖CarSim运行证据');copyfile(source,target);
 h=sha(target);assert(strcmp(h,sha(source)));[ok,msg]=fileattrib(target,'-w');assert(ok,'%s',msg);
 evidence.outputs.(keys{k})=struct('source',source,'saved',target,'sha256',h,'source_modified_datenum',info.datenum);
end
if isfile(paths.inputarchive)
 h=sha(paths.inputarchive);evidence.actual_expanded_sha256=h;
 if ~strcmp(h,c.source_sha256)
  target=fullfile(folder,'LastRun_all.par.evidence');assert(~isfile(target));copyfile(paths.inputarchive,target);fileattrib(target,'-w');
  evidence.mismatched_expanded_saved=target; % 保留配置错误，而非在失败处理时抹掉。
 end
end
end

function result=verify_archive(out,path,meta)
% 每个通道写后读回按double位模式逐点核对；NaN与±0原位保留，不做数值归一化。
names=fieldnames(meta.channels);checks=struct;
for k=1:numel(names)
 key=names{k};info=meta.channels.(key);item=load(path,key);saved=item.(key);
 if strcmp(info.source_field,'logsout')
  dataset=out.get('logsout');element=dataset.getElement(info.element_index);value=element.Values;
 else
  value=out.get(info.source_field);
 end
 if strcmp(info.source_field,'tout'),original=value(:);else,original=flatten_log(value);end
 same=isequal(size(saved),size(original))&&isequal(typecast(saved(:),'uint64'),typecast(original(:),'uint64'));
 checks.(key)=same;assert(same,'归档写后位模式变化:%s',key);
end
result=struct('all_equal',all(cell2mat(struct2cell(checks))),'checks',checks);
end

function z=flatten_log(value)
if isa(value,'timeseries')
 t=value.Time;data=value.Data;
 if value.IsTimeFirst
  a=reshape(data,numel(t),[]);
 else
  a=reshape(data,[],numel(t))';
 end
elseif isstruct(value)&&isfield(value,'time')&&isfield(value,'signals')
 t=value.time;data=value.signals.values;
 if size(data,1)==numel(t)
  a=reshape(data,numel(t),[]);
 else
  a=reshape(data,[],numel(t))';
 end
else
 error('需要独立审核的日志类型:%s',class(value));
end
if ~isa(a,'double'),assert(isa(a,'int8'));a=double(a);end
z=[t(:),a];
end

function proof=checkinputs(path,c,folder)
d=load(path,'asb_import12','asb_diag','asb_suspension','repair_plant_raw','repair_actuator_command');
imp=d.asb_import12;ad=d.asb_diag;sus=d.asb_suspension;raw=d.repair_plant_raw;
act=d.repair_actuator_command;
assert(size(imp,2)==13&&size(sus,2)==27&&size(raw,2)==51&&size(act,2)==9,...
 '实际接口宽度变化');
assert(isequal(imp(:,1),raw(:,1))&&isequal(sus(:,1),raw(:,1))&&...
 isequal(act(:,1),raw(:,1)),'车辆/导入/实际8路时轴变化');
% 原执行层断言时ASB诊断可能多一个终点事件；只比共同原生前缀并留末点证据。
assert(size(ad,1)>=size(raw,1)&&isequal(ad(1:size(raw,1),1),raw(:,1)),...
 'ASB诊断与车辆原生共同前缀时轴不符');
assert(numel(imp(:,1))>=2&&max(abs(diff(imp(:,1))-.0005))<1e-10);
assert(all(isfinite(imp(:,1:9)),'all'),'实际8路输入存在非有限数值');
assert(isequal(imp(:,2:9),act(:,2:9)),'导入CarSim的实际8路与执行层输出不一致');
assert(all(imp(:,10:13)==0,'all'),'实际导入外加ASB非零');
assert(all(ad(:,8:9)==0,'all')&&all(sus(:,6:9)==0,'all'),'外加ASB指令/实收FsExt非零');
text=fileread(fullfile(folder,'LastRun_echo.par.evidence'));actual=zeros(1,2);
for k=1:2
 token=regexp(text,sprintf('(?m)^MX_AUX_COEFFICIENT\\(%d\\)\\s+([-+0-9.eE]+)',k),'tokens');
 assert(numel(token)==1,'echo内AuxRoll系数缺失或重复');actual(k)=str2double(token{1}{1});
end
assert(isequal(actual,[c.kf_deg c.kr_deg]),'实际CarSim echo内部K不符');
auxres=sus(:,26:27)-sus(:,2:3).*[c.kf_deg c.kr_deg];
proof=struct('actual_internal_K_Nm_per_deg',actual,'external_imports_exact_numeric_zero',true,...
 'external_command_and_received_FsExt_zero',true,'actual8_equal_import8',true,...
 'native_dt_s',.0005,'samples',size(imp,1),...
 'asb_diag_samples_original',size(ad,1),'asb_diag_terminal_extra_events',size(ad,1)-size(raw,1),...
 'native_AuxMx_minus_K_relativePhi_peak_Nm',max(abs(auxres),[],1),'no_input_replay',true,...
 'state_and_physical_performance_not_thresholded_here',true);
end

function hash=checkfreeze(w)
% 冻结清单由主任务在编译审核后生成；逐文件核对本轮模型/状态机/执行源。
p=fullfile(w,'source','runner_freeze.json');assert(isfile(p),'缺少执行源冻结清单');
f=jsondecode(fileread(p));assert(f.preparation_reviewed&&f.compiled_and_reviewed,...
 '执行源尚未完成编译审核');
listed=cell(size(f.files));for k=1:numel(f.files),listed{k}=strrep(char(f.files(k).path),'\','/');end
essential={'model/stabilizer_bar_step37a.slx','model/a001_step37a_initialize.m',...
 'model/repair_support/repair_entry_bank.json','model/repair_support/repair_cycle_bank.json',...
 'model/repair_support/repair_stadium_reference.json',...
 'internal/state_cycle_core.m','internal/native_state_capture.m',...
 'internal/run_state_only_case.m','internal/prepare_state_configs.py',...
 'source/profiles/V12_beta30_state_only_R3_S.json',...
 'source/config_manifest.json','source/public_executor_frozen.txt'};
assert(all(ismember(essential,listed)),'执行源冻结清单缺少模型/初始化/状态机/两profile/runner');
helper=fullfile(fileparts(w),'step35a_beta30_20261006','internal','save_numeric_archive.m');
assert(strcmp(sha(helper),'5dd3289b2844d7f222159c3b58efe6e40fdb99f389aaa63cf030e41b2226a3de'),...
 '原生无损归档辅助函数已变化');
for k=1:numel(f.files)
 q=fullfile(w,f.files(k).path);assert(strcmp(sha(q),f.files(k).sha256),'研究执行源变化:%s',q);
end
if isfield(f,'external_files')
 for k=1:numel(f.external_files)
  q=f.external_files(k).path;assert(strcmp(sha(q),f.external_files(k).sha256),'外部保护源变化:%s',q);
 end
end
hash=sha(p);
end
function tf=samepath(a,b),tf=strcmpi(char(java.io.File(a).getCanonicalPath()),char(java.io.File(b).getCanonicalPath()));end
function h=sha(p)
f=fopen(p,'rb');assert(f>=0,'读取文件失败:%s',p);clean=onCleanup(@()fclose(f));bytes=fread(f,Inf,'*uint8');
md=java.security.MessageDigest.getInstance('SHA-256');if ~isempty(bytes),md.update(typecast(bytes,'int8'));end;h=lower(reshape(dec2hex(typecast(md.digest(),'uint8'),2)',1,[]));
end
function writejson(p,x),write_text(p,jsonencode(x));end
function write_text(p,x)
f=fopen(p,'w','n','UTF-8');assert(f>=0);clean=onCleanup(@()fclose(f));fprintf(f,'%s',x);
end
