
clear; close all;

% compute the transformation matrix for the registration
invTregister = eye(4);

position = [205 127 1363 971];
cameraPosition = [204.4461 -180.8787 -213.2714];

h = [];

% compute the transformation matrix for the first link of the i'th path
T = eye(4);

% draw the first link
h = [h, drawLinkSkeleton(T, 7.9730, 5.0)];
axis equal;
axis([-20 20 -20 20 -20 20]);
set(gcf,'Position',position);
set(gca,'CameraPosition',cameraPosition);
    
% extract phi and theta
phi = 0.3;
theta = 0.7 + pi;
            
% compute the transformation for phi and theta
Tapply(1,:) = [cos(phi), -sin(phi)*cos(theta-pi/2.0), -sin(phi)*sin(theta-pi/2.0), 0];
Tapply(2,:) = [sin(phi)*cos(theta-pi/2.0), cos(phi)*cos(theta-pi/2.0)*cos(theta-pi/2.0)+sin(theta-pi/2.0)*sin(theta-pi/2.0), cos(phi)*cos(theta-pi/2.0)*sin(theta-pi/2.0)-sin(theta-pi/2.0)*cos(theta-pi/2.0), 0];
Tapply(3,:) = [sin(phi)*sin(theta-pi/2.0), cos(phi)*sin(theta-pi/2.0)*cos(theta-pi/2.0)-cos(theta-pi/2.0)*sin(theta-pi/2.0), cos(phi)*sin(theta-pi/2.0)*sin(theta-pi/2.0)+cos(theta-pi/2.0)*cos(theta-pi/2.0), 0];
Tapply(4,:) = [0, 0, 0, 1];
            
% rotate the transformation matrix according to phi and theta            
T(1:3,1:3) = Tapply(1:3,1:3)*T(1:3,1:3);
            
% calculate the yaw and pitch values from the transformation matrix
rz = atan2(T(2,1), T(1,1));
ry = atan2(-T(3,1), sqrt(T(3,2)^2 + T(3,3)^2));
            
% move forward the position by LINK_LENGTH, keep the orientation the same
T3(1,1) = 3.5*7.9730;
T3(2,1) = 0;
T3(3,1) = 0;
T(1,4) = T(1,4) + cos(rz)*cos(ry)*7.9730;
T(2,4) = T(2,4) + sin(rz)*cos(ry)*7.9730;
T(3,4) = T(3,4) - sin(ry)*7.9730;
T2(1,1) = T(1,4) + 2.5*cos(rz)*cos(ry)*7.9730;
T2(2,1) = T(2,4) + 2.5*sin(rz)*cos(ry)*7.9730;
T2(3,1) = T(3,4) - 2.5*sin(ry)*7.9730;

           
% draw the link
h = [h, drawLinkSkeleton(invTregister*T, 7.9730, 5.0)];
hold on;

points = [0 T2(1,1) T2(1,1) 0; 0 T2(2,1) T3(2,1) 0; 0 T2(3,1) T3(3,1) 0];
%patch(points(1,:)',points(2,:)',points(3,:)',zeros(size(points(1,:)')),'FaceColor',[0.9 0.9 0.9],'FaceAlpha',1.0,'EdgeColor',[0 0 0],'EdgeAlpha',1.0);

theta = pi;
Tapply(1,:) = [cos(phi), -sin(phi)*cos(theta-pi/2.0), -sin(phi)*sin(theta-pi/2.0), 0];
Tapply(2,:) = [sin(phi)*cos(theta-pi/2.0), cos(phi)*cos(theta-pi/2.0)*cos(theta-pi/2.0)+sin(theta-pi/2.0)*sin(theta-pi/2.0), cos(phi)*cos(theta-pi/2.0)*sin(theta-pi/2.0)-sin(theta-pi/2.0)*cos(theta-pi/2.0), 0];
Tapply(3,:) = [sin(phi)*sin(theta-pi/2.0), cos(phi)*sin(theta-pi/2.0)*cos(theta-pi/2.0)-cos(theta-pi/2.0)*sin(theta-pi/2.0), cos(phi)*sin(theta-pi/2.0)*sin(theta-pi/2.0)+cos(theta-pi/2.0)*cos(theta-pi/2.0), 0];
Tapply(4,:) = [0, 0, 0, 1];
T = eye(4);
T(1:3,1:3) = Tapply(1:3,1:3)*T(1:3,1:3);
rz = atan2(T(2,1), T(1,1));
ry = atan2(-T(3,1), sqrt(T(3,2)^2 + T(3,3)^2));
T(1,4) = T(1,4) + 3.5*cos(rz)*cos(ry)*7.9730;
T(2,4) = T(2,4) + 3.5*sin(rz)*cos(ry)*7.9730;
T(3,4) = T(3,4) - 3.5*sin(ry)*7.9730;
%plot3([T2(1,1) T(1,4)],[T3(2,1) T(2,4)],[T3(3,1) T(3,4)],'k:','LineWidth',1.0);
%plot3([T2(1,1)],[T3(2,1)],[T3(3,1)],'k.');

points = [];
for theta = 0:5*pi/180:2*pi,
    Tapply(1,:) = [cos(phi), -sin(phi)*cos(theta-pi/2.0), -sin(phi)*sin(theta-pi/2.0), 0];
    Tapply(2,:) = [sin(phi)*cos(theta-pi/2.0), cos(phi)*cos(theta-pi/2.0)*cos(theta-pi/2.0)+sin(theta-pi/2.0)*sin(theta-pi/2.0), cos(phi)*cos(theta-pi/2.0)*sin(theta-pi/2.0)-sin(theta-pi/2.0)*cos(theta-pi/2.0), 0];
    Tapply(3,:) = [sin(phi)*sin(theta-pi/2.0), cos(phi)*sin(theta-pi/2.0)*cos(theta-pi/2.0)-cos(theta-pi/2.0)*sin(theta-pi/2.0), cos(phi)*sin(theta-pi/2.0)*sin(theta-pi/2.0)+cos(theta-pi/2.0)*cos(theta-pi/2.0), 0];
    Tapply(4,:) = [0, 0, 0, 1];
    T = eye(4);
    T(1:3,1:3) = Tapply(1:3,1:3)*T(1:3,1:3);
    rz = atan2(T(2,1), T(1,1));
    ry = atan2(-T(3,1), sqrt(T(3,2)^2 + T(3,3)^2));
    T(1,4) = T(1,4) + 3.5*cos(rz)*cos(ry)*7.9730;
    T(2,4) = T(2,4) + 3.5*sin(rz)*cos(ry)*7.9730;
    T(3,4) = T(3,4) - 3.5*sin(ry)*7.9730;
    points = [points, T(1:3,4)];    
end
%patch([points(1,:)'; points(1,1)],[points(2,:)'; points(2,1)],[points(3,:)'; points(3,1)],zeros(size([points(1,:)'; points(1,1)])),'FaceColor',[0.7 0.7 0.7],'FaceAlpha',0.4,'EdgeColor',[0 0 0],'EdgeAlpha',1.0);
    
    
    



