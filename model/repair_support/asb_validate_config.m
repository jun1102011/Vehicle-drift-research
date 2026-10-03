function asb_validate_config(path)
% 核对本轮审阅后的CarSim输入：只忽略导出时间注释，不自动放行物理参数改变。
root=fileparts(mfilename('fullpath'));approved=fileread(fullfile(root,'asb_approved_Run_all.par.evidence'));
current=fileread(path);strip=@(s) regexprep(s,'(?m)^! File created:[^\r\n]*','');
assert(strcmp(strip(current),strip(approved)), ...
 'ASB:ConfigChanged','CarSim与本轮已审阅的12输入/76输出配置不同，请先审查差异；禁止自动套用旧配平结论。');
end
