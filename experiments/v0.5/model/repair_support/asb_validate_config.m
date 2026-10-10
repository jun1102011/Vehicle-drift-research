function asb_validate_config(path)
% Step33d文件派生配置校验：原GUI导出基线，仅两条AuxRoll系数与sim输入/输出路径允许变化。
% approved证据用于核对，不作为运行输入；旧控制增益/工作点来源不改。
assert(evalin('base','exist(''aux_case_cfg'',''var'')')==1,'未选择已审核GUI配置');
c=evalin('base','aux_case_cfg');needed={'case','source_path','source_sha256','approved_evidence','semantic_review_path','kf_deg','kr_deg','simfile_path','simfile_sha256','sim_review_path','approved_sim_evidence','configuration_mode','file_provenance_path','file_provenance_sha256'};
assert(all(isfield(c,needed)),'逐case配置身份不完整');
assert(strcmp(c.configuration_mode,'file_derived_from_GUI_baseline'),'本研究副本只接受独立文件派生配置');
p=jsondecode(fileread(c.file_provenance_path));
assert(strcmp(sha(c.file_provenance_path),c.file_provenance_sha256),'文件派生来源记录已变化');
assert(strcmp(p.schema,'step33d-file-derived-v1') && strcmp(p.case,c.case) && strcmp(p.configuration_mode,c.configuration_mode),'来源方式或case不符');
assert(~p.GUI_database_mutated && isempty(p.new_GUI_dataset_UUIDs) && ~p.controller_banks_recomputed,'文件派生不得冒称GUI修改或控制银行重算');
assert(strcmp(p.baseline_source.sha256,'2b045ad9fd00a4f5f685b4ce6f603b1f464691a00beb194c792ae9deb2717fd6') && strcmp(sha(p.baseline_source.path),p.baseline_source.sha256),'原GUI参数来源不符');
assert(strcmp(p.baseline_sim.sha256,'7fd5643e30b21017494748d6774fb35f4254fdb936c9807ae57a986be2920460') && strcmp(sha(p.baseline_sim.path),p.baseline_sim.sha256),'原GUI sim来源不符');
assert(strcmp(p.source_sha256,c.source_sha256) && strcmp(p.sim_sha256,c.simfile_sha256) && p.kf_deg==c.kf_deg && p.kr_deg==c.kr_deg,'来源系数或SHA不符');
assert(p.exact_byte_review.passed && p.exact_byte_review.source_non_target_bytes_identical && p.exact_byte_review.sim_non_target_bytes_identical,'派生文件存在非目标变化');
assert(strcmp(path,c.source_path) && isfile(path) && isfile(c.approved_evidence),'实际配置路径与本组身份不一致');
r=jsondecode(fileread(c.semantic_review_path));
assert(strcmp(r.schema,'step33d-exact-expanded-diff-v1') && r.passed && isempty(r.errors) && isempty(r.unresolved_differences),'该GUI配置未通过完整非目标差异审查');
assert(r.non_target_bytes_identical && strcmp(r.configuration_mode,c.configuration_mode),'源参数存在非目标字节变化');
assert(strcmp(r.baseline.sha256,'2b045ad9fd00a4f5f685b4ce6f603b1f464691a00beb194c792ae9deb2717fd6'),'原批准配置身份错误');
assert(isequal(r.Kaux_front_rear_Nm_per_deg(:),[c.kf_deg;c.kr_deg]),'核准系数与case不一致');
assert(strcmp(sha(c.approved_evidence),r.candidate.sha256) && strcmp(r.candidate.sha256,c.source_sha256),'批准证据已有变化，先重新审查');
names={'kf384_kr384','kf384_kr192','kf192_kr384','kf576_kr384','kf384_kr576'};pairs=[384 384;384 192;192 384;576 384;384 576];
idx=find(strcmp(names,c.case),1);assert(~isempty(idx) && isequal(pairs(idx,:),[c.kf_deg c.kr_deg]),'case名称与授权组合不符');
sr=jsondecode(fileread(c.sim_review_path));
assert(strcmp(sr.schema,'step33d-exact-sim-diff-v1') && sr.passed && strcmp(sr.case,c.case) && sr.full_commands_reviewed && sr.nonphysical_diff_review_complete,'sim全部指令未通过逐项审查');
assert(sr.non_target_bytes_identical && strcmp(sr.configuration_mode,c.configuration_mode) && strcmp(sr.baseline_sha256,p.baseline_sim.sha256),'sim模式或非目标字节不符');
for j=1:numel(sr.differences)
    assert(sr.differences(j).approved && sr.differences(j).nonphysical_only && ~isempty(strtrim(sr.differences(j).rationale)),'sim存在未明确审阅的差异');
end
assert(strcmp(sr.candidate_sha256,c.simfile_sha256) && strcmp(sr.input_sha256,c.source_sha256),'sim审核身份与实际配置不符');
assert(strcmp(sha(c.simfile_path),c.simfile_sha256),'实际sim文件已变化，先重新审查');
assert(strcmp(sha(c.approved_sim_evidence),c.simfile_sha256),'批准的只读sim证据已变化');
assert(strcmp(sha(path),c.source_sha256),'实际GUI展开配置已有变化，禁止静默套用');
strip=@(s)regexprep(s,'(?m)^! File created:[^\r\n]*','');
assert(strcmp(strip(fileread(path)),strip(fileread(c.approved_evidence))),...
 '本组实际GUI配置与独立批准证据不符；禁止跳过审核或重算控制银行');
end
function h=sha(p)
f=fopen(p,'rb');assert(f>=0);clean=onCleanup(@()fclose(f));bytes=fread(f,Inf,'*uint8'); %#ok<NASGU>
md=java.security.MessageDigest.getInstance('SHA-256');md.update(typecast(bytes,'int8'));h=lower(reshape(dec2hex(typecast(md.digest(),'uint8'),2)',1,[]));
end
