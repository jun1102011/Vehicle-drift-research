function test_logic()
% 纯公式核验：双向600原生点、相同状态下世界坐标不影响请求、固定维持窗口。
w=fileparts(fileparts(mfilename('fullpath')));d=fullfile(w,'audit/logic_functions');if ~isfolder(d),mkdir(d);end
for j=1:2
 for k=1:2
  if k==1,src='state_cycle_core.m';old='cycle_control';name=['testcore' num2str(j)];else,src='native_state_capture.m';old='stadium_guidance';name=['testnative' num2str(j)];end
  s=strrep(fileread(fullfile(w,'internal',src)),['=' old '('],['=' name '(']);f=fopen(fullfile(d,[name '.m']),'w','n','UTF-8');fprintf(f,'%s',s);fclose(f);
 end
end
addpath(d);clear testcore1 testcore2 testnative1 testnative2
p=jsondecode(fileread(fullfile(w,'source/profiles/LeftRight_R3.json')));entry=jsondecode(fileread(fullfile(w,'model/repair_support/repair_entry_bank.json')));entry.normal=p.entry_normal;entry.nodes=p.entry_nodes;entry.slip_high(2)=Inf;
cycle=jsondecode(fileread(fullfile(w,'model/repair_support/repair_cycle_bank.json')));cycle.nodes=p.exit_nodes;state=p.state_protocol;state.target=state.target(:);state.right_target=state.right_target(:);state.entry_tolerance=state.entry_tolerance(:);repair=struct('Ts',.01,'Rw',.32,'Iw',1.5);st=struct('disabled',1);
for side=1:2
 clear testnative1
 if side==1,x=entry.nodes(end).x;else,x=state.right_entry_nodes(end).x;end
 for k=0:599
  pa=testnative1(make_y(x),k*.0005,st,entry,repair,state);
  index=1;if side==2,index=6;end
  assert(pa(index)==double(k==599));
 end
end
clear testcore1 testcore2 testnative1 testnative2
ph=zeros(6001,1);mx=0;
for k=0:120000
 t=k*.0005;
 if t<3||t>=48,x=entry.nodes(1).x;elseif t<16,x=entry.nodes(end).x;else,x=state.right_entry_nodes(end).x;end
 ya=make_y(x);yb=ya;yb(35:37)=[1e4*sin(t);1e4*cos(t);179*sin(t)];
 pa=testnative1(ya,t,st,entry,repair,state);pb=testnative2(yb,t,st,entry,repair,state);assert(isequal(pa,pb));
 if mod(k,20)==0
  [a,da,ca,sa]=testcore1(ya,t,entry,repair,cycle,pa,state);[b,db,cb,sb]=testcore2(yb,t,entry,repair,cycle,pb,state);
  mx=max(mx,max(abs([a-b;da-db;ca-cb;sa-sb])));ph(k/20+1)=ca(1);
 end
end
assert(mx==0);assert(sum(ph==3)==1000&&sum(ph==9)==1000&&any(ph==7)&&any(ph==12));assert(~any(ismember(ph,[4 5 6])));
r=struct('passed',true,'native600_both_directions',true,'no_path_command_difference',mx,'left_hold_steps',sum(ph==3),'right_hold_steps',sum(ph==9),'no_straight_dwell_stage',true,'scope','synthetic logic; not vehicle performance');f=fopen(fullfile(w,'audit/logic_validation.json'),'w','n','UTF-8');fprintf(f,'%s',jsonencode(r));fclose(f);disp('STEP37A_LOGIC_PASS');
end
function y=make_y(x)
y=zeros(50,1);y(2)=x(1)*cos(x(2))*3.6;y(3)=x(1)*sin(x(2))*3.6;y(4)=x(2)*180/pi;y(5)=x(3)*180/pi;y(8)=x(6)*180/pi;y(26)=x(7)*180/pi;y(17:20)=4500;y(27:30)=[x(4);x(4);x(5);x(5)];y(21:24)=12/.32*(1+y(27:30))*30/pi;
end
