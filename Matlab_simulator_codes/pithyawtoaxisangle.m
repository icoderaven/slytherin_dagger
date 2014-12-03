function [phi,theta]=pithyawtoaxisangle(pitch,yaw)
rz=pitch;
ry=yaw;
R(1,1:3) = [cos(ry)*cos(rz), -sin(rz),sin(ry)*cos(rz)];
R(2,1:3) = [cos(ry)*sin(rz), cos(rz),sin(ry)*sin(rz)];
R(3,1:3) = [-sin(ry),0, cos(ry)];

phi=atan2(sqrt(R(2,1)^2+R(3,1)^2),R(1,1));
if phi==0
    theta=0;
else
    theta=pi/2.0+atan2(R(3,1)/sin(phi),R(2,1)/sin(phi));
end


end