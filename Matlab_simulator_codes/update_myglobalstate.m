function update_myglobalstate(ctrl_msg)
global global_state;

display('[update_myglobalstate]: updating global_state')

tmp = ctrl_msg.getLinear();
yaw = tmp.getX;
pitch = tmp.getY;

[phi,theta]=pithyawtoaxisangle(pitch,yaw);
global_state(end-1)=phi;
global_state(end)=theta;
global_state=adderror(global_state,0);

global_state=adderror(global_state,1);
if length(global_state)<66
global_state=[global_state,0,0];
end
