function independent_state_harness()
% Independent pure-MATLAB checks. No model, DLL, or vehicle is opened.
w=fileparts(fileparts(mfilename('fullpath')));
outdir=fullfile(w,'audit','independent_harness_functions');if ~isfolder(outdir),mkdir(outdir);end
corepath=fullfile(w,'internal','state_cycle_core.m');nativepath=fullfile(w,'internal','native_state_capture.m');
core=fileread(corepath);native=fileread(nativepath);
names={'independent_core_a','independent_core_b','independent_native_a','independent_native_b'};
for k=1:4
 if k<=2,txt=strrep(core,'=cycle_control(',['=' names{k} '(']);else,txt=strrep(native,'=stadium_guidance(',['=' names{k} '(']);end
 fid=fopen(fullfile(outdir,[names{k} '.m']),'w','n','UTF-8');assert(fid>=0);fwrite(fid,txt,'char');fclose(fid);
end
addpath(outdir);cleanpath=onCleanup(@()rmpath(outdir)); %#ok<NASGU>
clear independent_core_a independent_core_b independent_native_a independent_native_b
p=jsondecode(fileread(fullfile(w,'source','profiles','V12_beta30_state_only_R3.json')));
entry=jsondecode(fileread(fullfile(w,'model','repair_support','repair_entry_bank.json')));
cycle=jsondecode(fileread(fullfile(w,'model','repair_support','repair_cycle_bank.json')));
entry.normal=p.entry_normal;entry.nodes=p.entry_nodes;entry.slip_high(2)=Inf;cycle.nodes=p.exit_nodes;
state=p.state_protocol;state.target=state.target(:);state.entry_tolerance=state.entry_tolerance(:);
repair=struct('Ts',.01,'Rw',.32,'Iw',1.5);stadium=struct('disabled',1);
y=make_y(entry.nodes(end).x,repair.Rw);
% Exactly 599/600 native samples, duplicate invocation, and one failed sample.
for k=0:598
 [a,~,~,~]=independent_native_a(y,k*.0005,stadium,entry,repair,state);
 assert(a(1)==0&&a(4)==k+1,'Native event triggered before 600 samples');
end
[dup,~,~,~]=independent_native_a(y,598*.0005,stadium,entry,repair,state);
assert(isequal(a,dup),'Duplicate invocation changed native event state');
[a,~,~,~]=independent_native_a(y,599*.0005,stadium,entry,repair,state);
assert(a(1)==1&&a(4)==600&&abs(a(2)-.2995)<1e-12&&abs(a(3))<1e-12);
clear independent_native_a
for k=0:1199
 yy=y;if k==599,yy(4)=0;end
 [a,~,~,~]=independent_native_a(yy,k*.0005,stadium,entry,repair,state);
 if k<1199,assert(a(1)==0,'Interrupted native hold incorrectly latched');end
end
assert(a(1)==1&&abs(a(2)-.5995)<1e-12&&abs(a(3)-.3)<1e-12);
clear independent_native_a
sparse_rejected=false;sparse_latched=false;
try
 for k=0:599
  [a,~,~,~]=independent_native_a(y,k*.01,stadium,entry,repair,state);
  sparse_latched=sparse_latched||(a(1)>0);
 end
catch err
 sparse_rejected=true;sparse_message=err.message;
end
if ~sparse_rejected,sparse_message='No assertion; sparse input did not latch';end
assert(sparse_rejected||~sparse_latched,'0.01 s calls incorrectly counted as 600 native samples');
clear independent_core_a independent_core_b independent_native_a independent_native_b
% Identical physical states, different X/Y/Yaw; compare both independent histories.
max_command=0;max_diag=0;max_packet=0;tc=[];phase=[];
for k=0:70000
 t=k*.0005;
 if t<3||t>=25,x=entry.nodes(1).x;else,x=entry.nodes(end).x;end
 ya=make_y(x,repair.Rw);yb=ya;
 ya(35:37)=[0;0;0];yb(35:37)=[10000*sin(t);10000*cos(t);179*sin(.3*t)];
 [pa,sa,ra,fa]=independent_native_a(ya,t,stadium,entry,repair,state);
 [pb,sb,rb,fb]=independent_native_b(yb,t,stadium,entry,repair,state);
 max_packet=max(max_packet,max(abs([pa-pb;sa-sb;ra-rb;fa-fb])));
 if mod(k,20)==0
  [ca,da,cd_a]=independent_core_a(ya,t,entry,repair,cycle,pa,state);
  [cb,db,cd_b]=independent_core_b(yb,t,entry,repair,cycle,pb,state);
  max_command=max(max_command,max(abs(ca-cb)));
  max_diag=max(max_diag,max(abs([da-db;cd_a-cd_b])));
  tc(end+1)=t;phase(end+1)=cd_a(1); %#ok<AGROW>
 end
end
assert(max_command==0&&max_diag==0&&max_packet==0,'X/Y/Yaw changed task signals');
i=find(phase==3);assert(numel(i)==1000&&all(diff(i)==1),'Hold is not one fixed 1000-control-step interval');
j=find(phase==4,1);assert(~isempty(j)&&abs(tc(j)-tc(i(1))-10)<1e-10,'Exit did not start exactly 10 s after first hold');
result=struct('passed',true,'no_vehicle_or_model_opened',true,'native_599_600_boundary',true,...
 'duplicate_native_invocation_no_increment',true,'one_bad_sample_resets_run',true,...
 'sparse_001s_calls_rejected_or_not_latched',true,'sparse_assertion',sparse_message,...
 'world_XY_Yaw_perturbation_command_max_abs',max_command,'all_diag_max_abs',max_diag,...
 'native_packet_max_abs',max_packet,'fixed_hold_control_samples',numel(i),...
 'hold_start_s',tc(i(1)),'exit_start_s',tc(j),'core_sha256',hashfile(corepath),...
 'native_sha256',hashfile(nativepath),'scope','Pure logic only; does not establish nonlinear vehicle feasibility.');
fid=fopen(fullfile(w,'audit','independent_matlab_harness.json'),'w','n','UTF-8');assert(fid>=0);fprintf(fid,'%s',jsonencode(result));fclose(fid);
disp('INDEPENDENT_STATE_HARNESS_PASS');
end

function y=make_y(x,Rw)
y=zeros(50,1);y(2)=x(1)*cos(x(2))*3.6;y(3)=x(1)*sin(x(2))*3.6;
y(4)=x(2)*180/pi;y(5)=x(3)*180/pi;y(8)=x(6)*180/pi;y(26)=x(7)*180/pi;
y(17:20)=4500;y(27:30)=[x(4);x(4);x(5);x(5)];
y(21:24)=12/Rw*(1+y(27:30))*30/pi;
end

function h=hashfile(p)
f=fopen(p,'rb');assert(f>=0);b=fread(f,Inf,'*uint8');fclose(f);
m=java.security.MessageDigest.getInstance('SHA-256');m.update(typecast(b,'int8'));
h=lower(reshape(dec2hex(typecast(m.digest(),'uint8'),2).',1,[]));
end
