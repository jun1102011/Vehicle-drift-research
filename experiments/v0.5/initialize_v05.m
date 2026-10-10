% V0.5发布选择入口：仅准备原模型参数，不启动车辆，不改sim/par。
% v05_case为前/后辅助刚度身份；原唯一核心初始化仍为a001_step37a_initialize。
if ~exist('v05_case','var'), v05_case='kf384_kr384'; end
v05_root=fileparts(mfilename('fullpath'));
assert(any(strcmp(v05_case,{'kf384_kr384','kf192_kr384','kf576_kr384','kf384_kr192','kf384_kr576'})),'未知V0.5刚度');
aux_case_cfg=jsondecode(fileread(fullfile(v05_root,'source','file_cases',v05_case,'aux_case_cfg.json')));
% 五组都使用已核查的Step37c直线道路参考坐标；不改变物理平面。
aux_case_cfg.schema='step37a-state-only-file-case-v1';
for v05_pair={'source','simfile'}
 v05_key=v05_pair{1};v05_path=aux_case_cfg.([v05_key '_path']);
 assert(isfile(v05_path),'原机批准配置缺失：请按README迁移核验，不能跳过');
 v05_fid=fopen(v05_path,'rb');v05_bytes=fread(v05_fid,Inf,'*uint8');fclose(v05_fid);
 v05_md=java.security.MessageDigest.getInstance('SHA-256');v05_md.update(typecast(v05_bytes,'int8'));
 v05_hash=lower(reshape(dec2hex(typecast(v05_md.digest(),'uint8'),2)',1,[]));
 assert(strcmp(v05_hash,aux_case_cfg.([v05_key '_sha256'])),'批准配置哈希改变，请核查用户新修改');
end
profile_id='LeftRight_R3';asb_case='baseline';
asb_pulse_cfg=struct('moment_Nm',[0;0],'start_s',0,'end_s',0,'edge_s',.1);
addpath(fullfile(v05_root,'model'));addpath(fullfile(v05_root,'internal'));
run(fullfile(v05_root,'model','a001_step37a_initialize.m'));
fprintf('V0.5参数已准备；尚未启动仿真。\n');
