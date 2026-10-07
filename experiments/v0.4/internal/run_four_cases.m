function run_four_cases()
% 只串行运行已授权四个刚度；运行器每组重新核对配置/源/接口并归档。
% 性能验收不作为下一组放行条件；接口、源或车辆异常返回错误时先停止审查。
w=fileparts(fileparts(mfilename('fullpath')));cases={'kf192_kr384','kf576_kr384','kf384_kr192','kf384_kr576'};done={};
for k=1:numel(cases)
 write_status(w,'RUNNING',done,cases{k});
 run_state_only_case('V12_beta30_state_only_R3',cases{k});
 done{end+1}=cases{k}; %#ok<AGROW>
 write_status(w,'ARCHIVED_PENDING_OFFLINE_REVIEW',done,'');
 fprintf('STEP35G_CASE_DONE %d/4 %s\n',k,cases{k});
end
end
function write_status(w,state,done,current)
p=fullfile(w,'batch_status.json');f=fopen(p,'w','n','UTF-8');assert(f>=0);c=onCleanup(@()fclose(f));fprintf(f,'%s',jsonencode(struct('state',state,'completed',{done},'current_case',current,'new_runs',numel(done),'historical_baseline_referenced',true)));
end
