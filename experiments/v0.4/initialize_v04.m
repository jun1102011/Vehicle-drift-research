% V0.4发布选择入口：仅准备原模型参数，不启动车辆，不改sim/par。
% v04_case为前/后辅助刚度身份；原唯一核心初始化仍为a001_step35g_initialize。
if ~exist('v04_case','var'), v04_case='kf384_kr384'; end
v04_root=fileparts(mfilename('fullpath'));
assert(any(strcmp(v04_case,{'kf384_kr384','kf192_kr384','kf576_kr384','kf384_kr192','kf384_kr576'})),'未知V0.4刚度');
aux_case_cfg=jsondecode(fileread(fullfile(v04_root,'source','file_cases',v04_case,'aux_case_cfg.json')));
% 384基线来自Step35f；只统一内存身份给相同Step35g初始化，物理文件和控制数值不变。
aux_case_cfg.schema='step35g-state-only-file-case-v1';
for v04_pair={'source','simfile'}
 v04_key=v04_pair{1};v04_path=aux_case_cfg.([v04_key '_path']);
 assert(isfile(v04_path),'原机批准配置缺失：请按README迁移核验，不能跳过');
 v04_fid=fopen(v04_path,'rb');v04_bytes=fread(v04_fid,Inf,'*uint8');fclose(v04_fid);
 v04_md=java.security.MessageDigest.getInstance('SHA-256');v04_md.update(typecast(v04_bytes,'int8'));
 v04_hash=lower(reshape(dec2hex(typecast(v04_md.digest(),'uint8'),2)',1,[]));
 assert(strcmp(v04_hash,aux_case_cfg.([v04_key '_sha256'])),'批准配置哈希改变，请核查用户新修改');
end
profile_id='V12_beta30_state_only_R3';asb_case='baseline';
asb_pulse_cfg=struct('moment_Nm',[0;0],'start_s',0,'end_s',0,'edge_s',.1);
addpath(fullfile(v04_root,'model'));addpath(fullfile(v04_root,'internal'));
run(fullfile(v04_root,'model','a001_step35g_initialize.m'));
fprintf('V0.4参数已准备；尚未启动仿真。\n');
