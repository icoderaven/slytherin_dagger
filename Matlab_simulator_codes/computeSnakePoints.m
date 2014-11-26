function [snakePoints] = computeSnakePoints(state,LINK_LENGTH,Tregister)
% function [snakePoints] = computeSnakePoints(state,LINK_LENGTH,Tregister)

% extract information for the transformation matrix
qx = state(1); qy = state(2); qz = state(3);
rz = state(4); ry = state(5); rx = state(6);

% compute the transformation matrix for the first link
T = eye(4); T(1,4) = qx; T(2,4) = qy; T(3,4) = qz;
T(1,1:3) = [cos(ry)*cos(rz), -cos(rx)*sin(rz)+sin(rx)*sin(ry)*cos(rz), sin(rx)*sin(rz)+cos(rx)*sin(ry)*cos(rz)];
T(2,1:3) = [cos(ry)*sin(rz), cos(rx)*cos(rz)+sin(rx)*sin(ry)*sin(rz), -sin(rx)*cos(rz)+cos(rx)*sin(ry)*sin(rz)];
T(3,1:3) = [-sin(ry), sin(rx)*cos(ry), cos(rx)*cos(ry)];

% draw the first link
snakePoints = [];
point1 = Tregister*T*[-LINK_LENGTH; 0; 0; 1];   
point2 = Tregister*T*[0; 0; 0; 1];
for u = 0:0.1:1.0,
    snakePoints = [snakePoints, [(1-u).*point1 + u.*point2]];
end

% compute the number of extra segments
numAdditionalSegments = (length(state)-6)/2;
for j = 1:numAdditionalSegments,
    
    % extract phi and theta
    phi = state(2*j+5);
    theta = state(2*j+6);
    
    Tapply(1,:) = [cos(phi), -sin(phi)*cos(theta-pi/2.0), -sin(phi)*sin(theta-pi/2.0), 0];
    Tapply(2,:) = [sin(phi)*cos(theta-pi/2.0), cos(phi)*cos(theta-pi/2.0)*cos(theta-pi/2.0)+sin(theta-pi/2.0)*sin(theta-pi/2.0), cos(phi)*cos(theta-pi/2.0)*sin(theta-pi/2.0)-sin(theta-pi/2.0)*cos(theta-pi/2.0), 0];
    Tapply(3,:) = [sin(phi)*sin(theta-pi/2.0), cos(phi)*sin(theta-pi/2.0)*cos(theta-pi/2.0)-cos(theta-pi/2.0)*sin(theta-pi/2.0), cos(phi)*sin(theta-pi/2.0)*sin(theta-pi/2.0)+cos(theta-pi/2.0)*cos(theta-pi/2.0), 0];
    Tapply(4,:) = [0, 0, 0, 1];
    
    % rotate the transformation matrix according to phi and theta
    T(1:3,1:3) = T(1:3,1:3)*Tapply(1:3,1:3);
    
    % calculate the yaw and pitch values from the transformation matrix
    rz = atan2(T(2,1), T(1,1));
    ry = atan2(-T(3,1), sqrt(T(3,2)^2 + T(3,3)^2));
    
    % move forward the position by LINK_LENGTH, keep the orientation the same
    T(1,4) = T(1,4) + cos(rz)*cos(ry)*LINK_LENGTH;
    T(2,4) = T(2,4) + sin(rz)*cos(ry)*LINK_LENGTH;
    T(3,4) = T(3,4) - sin(ry)*LINK_LENGTH;
    
    % draw the link
    point1 = Tregister*T*[-LINK_LENGTH; 0; 0; 1];   
    point2 = Tregister*T*[0; 0; 0; 1];
    for u = 0:0.1:1.0,
        snakePoints = [snakePoints, [(1-u).*point1 + u.*point2]];
    end
end
snakePoints = snakePoints(1:3,:)';
