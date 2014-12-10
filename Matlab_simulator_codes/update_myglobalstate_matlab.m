function update_myglobalstate_matlab(yaw,pitch)
global global_state;

display('[update_myglobalstate]: updating global_state')

[phi,theta]=pithyawtoaxisangle(pitch,yaw);
global_state(end-1)=phi;
global_state(end)=theta;
global_state=adderror(global_state,0);

global_state=adderror(global_state,1);
if length(global_state)<66
    global_state=[global_state,0,0];
end
end