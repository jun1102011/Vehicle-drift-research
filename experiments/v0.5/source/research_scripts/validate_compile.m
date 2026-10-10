function validate_compile()
% 只初始化/编译已冻结副本，不保存模型、不启动60s车辆任务。
w=fileparts(fileparts(mfilename('fullpath')));md=fullfile(w,'model');m='stabilizer_bar_step37a';
cd(md);addpath(fullfile(w,'internal'));set(groot,'defaultFigureVisible','off');
cases={'kf384_kr384','kf192_kr384','kf576_kr384','kf384_kr192','kf384_kr576'};
load_system(fullfile(md,[m '.slx']));
for j=1:numel(cases)
 assignin('base','profile_id','LeftRight_R3');assignin('base','aux_case_cfg',jsondecode(fileread(fullfile(w,'source/file_cases',cases{j},'aux_case_cfg.json'))));
 assignin('base','asb_case','baseline');assignin('base','asb_pulse_cfg',struct('moment_Nm',[0;0],'start_s',0,'end_s',0,'edge_s',.1));
 evalin('base',['run(''' fullfile(md,'a001_step37a_initialize.m') ''');']);
 set_param(m,'SimulationCommand','update');
 actual=struct('entry',evalin('base','entry_cfg'),'cycle',evalin('base','cycle_cfg'),'repair',evalin('base','repair_cfg'),'state',evalin('base','state_cfg'),'path',evalin('base','path_cfg'));
 if j==1,base=actual;else,assert(isequaln(actual,base),'更换刚度改变了控制参数');end
 assert(actual.state.bidirectional==1&&abs(actual.state.reversal_s-29/3)<1e-12&&actual.state.path_enabled==0);
end
rt=sfroot;cs=rt.find('-isa','Stateflow.EMChart');found=0;
feval(m,[],[],[],'compile');
for k=1:numel(cs)
 if ~startsWith(cs(k).Path,[m '/']),continue;end
 if contains(cs(k).Script,'=cycle_control('),assert(isequal(get_param(cs(k).Path,'CompiledSampleTime'),[.01 0]));found=found+1;end
 if contains(cs(k).Script,'=stadium_guidance('),assert(isequal(get_param(cs(k).Path,'CompiledSampleTime'),[.0005 0]));found=found+1;end
end
feval(m,[],[],[],'term');assert(found==2);close_system(m,0);
r=struct('passed',true,'five_configurations_compiled',true,'controller_equal_all_cases',true,'model_not_saved',true,'vehicle_started',false,'control_step_s',.01,'native_step_s',.0005);
f=fopen(fullfile(w,'audit/compile_validation.json'),'w','n','UTF-8');fprintf(f,'%s',jsonencode(r));fclose(f);disp('STEP37B_COMPILE_PASS');
end
