function run_remaining()
% 仅按已授权顺序执行其余四组。每组独立保存、离线审核后才进入下一组。
w=fileparts(fileparts(mfilename('fullpath')));
python='C:/Users/28583/.cache/codex-runtimes/codex-primary-runtime/dependencies/python/python.exe';
cases={'kf192_kr384','kf576_kr384','kf384_kr192','kf384_kr576'};
for j=1:numel(cases)
 run_state_only_case('LeftRight_R3',cases{j});
 command=sprintf('"%s" -X utf8 "%s" %s',python,fullfile(w,'internal/review_case.py'),cases{j});
 [code,msg]=system(command);fprintf('%s\n',msg);assert(code==0,'离线技术审核失败，停止后续车辆运行');
end
disp('STEP37B_FIVE_TASKS_DONE');
end
