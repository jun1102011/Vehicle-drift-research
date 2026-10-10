function validate_publication()
% 只检查发布入口、冻结数值和编译，不调用sim，不保存模型。
root='E:/00硕士毕业论文/GitHub发布/stabilizer-bar-drift/experiments/v0.5';
cd(root);cases={'kf384_kr384','kf192_kr384','kf576_kr384','kf384_kr192','kf384_kr576'};
m='stabilizer_bar_step37a';
for i=1:numel(cases)
 assignin('base','v05_case',cases{i});evalin('base',['run(''' root '/initialize_v05.m'');']);
 if i==1,load_system(fullfile(root,'model',[m '.slx']));end
 set_param(m,'SimulationCommand','update');
 current=struct('entry',evalin('base','entry_cfg'),'cycle',evalin('base','cycle_cfg'),'repair',evalin('base','repair_cfg'),'state',evalin('base','state_cfg'),'path',evalin('base','path_cfg'));
 if i==1,reference=current;else,assert(isequaln(current,reference),'五配置控制数值不同');end
 assert(current.state.bidirectional==1 && current.state.path_enabled==0);
 assert(abs(current.state.reversal_s-29/3)<1e-12);
end
feval(m,[],[],[],'compile');feval(m,[],[],[],'term');close_system(m,0);
r=struct('passed',true,'five_initializations',5,'five_model_updates',5,'full_compile',true,'same_controller_all_cases',true,'vehicle_simulations',0,'model_saved',false);
f=fopen(fullfile(root,'evidence/publication_validation.json'),'w','n','UTF-8');fprintf(f,'%s',jsonencode(r));fclose(f);
disp('V05_PUBLICATION_COMPILE_PASS');
end
