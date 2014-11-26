function [Tin] = stateToTransformationMatrix(X, linkLength)
% function [Tin] = stateToTransformationMatrix(X, linkLength)

% compute the number of links
numLinks = (length(X)-6)/2;

% extract rz, ry, and rx
rz = X(4,1); ry = X(5,1); rx = X(6,1);

% fill in the appropriate values into the matrix
Tin = eye(4);
Tin(1,1) = cos(rz)*cos(ry);
Tin(1,2) = -sin(rz)*cos(rx)+cos(rz)*sin(ry)*sin(rx);
Tin(1,3) = sin(rz)*sin(rx)+cos(rz)*sin(ry)*cos(rx);
Tin(2,1) = sin(rz)*cos(ry);
Tin(2,2) = cos(rz)*cos(rx)+sin(rz)*sin(ry)*sin(rx);
Tin(2,3) = -cos(rz)*sin(rx)+sin(rz)*sin(ry)*cos(rx);
Tin(3,1) = -sin(ry);
Tin(3,2) = cos(ry)*sin(rx);
Tin(3,3) = cos(ry)*cos(rx);
Tin(1:3,4) = X(1:3);

% iterate through each link
for i = 1:numLinks,
    
    % extract the current angles
    phi = X(2*(i-1)+7);
	theta = X(2*(i-1)+8);
    
    % compute the turning transformation
    TapplyPhiTheta(1,1) =  cos(phi);
	TapplyPhiTheta(1,2) = -sin(phi)*cos(theta-pi/2.0);
	TapplyPhiTheta(1,3) = -sin(phi)*sin(theta-pi/2.0);%arun changed to (1,3)
	TapplyPhiTheta(2,1) =  sin(phi)*cos(theta-pi/2.0);
	TapplyPhiTheta(2,2) =  cos(phi)*cos(theta-pi/2.0)*cos(theta-pi/2.0)+sin(theta-pi/2.0)*sin(theta-pi/2.0);
	TapplyPhiTheta(2,3) =  cos(phi)*cos(theta-pi/2.0)*sin(theta-pi/2.0)-sin(theta-pi/2.0)*cos(theta-pi/2.0);
	TapplyPhiTheta(3,1) =  sin(phi)*sin(theta-pi/2.0);
	TapplyPhiTheta(3,2) =  cos(phi)*sin(theta-pi/2.0)*cos(theta-pi/2.0)-cos(theta-pi/2.0)*sin(theta-pi/2.0);
	TapplyPhiTheta(3,3) =  cos(phi)*sin(theta-pi/2.0)*sin(theta-pi/2.0)+cos(theta-pi/2.0)*cos(theta-pi/2.0);
	TapplyPhiTheta(4,4) =  1.0;
    
    % turn the link
    Tin = Tin*TapplyPhiTheta;
    
    % compute the angles
    rz = atan2(Tin(2,1), Tin(1,1));
	ry = atan2(-Tin(3,1), sqrt(Tin(3,2)^2+Tin(3,3)^2));

    % move the link forward
	Tin(1,4) = Tin(1,4) + cos(rz)*cos(ry)*linkLength;
	Tin(2,4) = Tin(2,4) + sin(rz)*cos(ry)*linkLength;
	Tin(3,4) = Tin(3,4) - sin(ry)*linkLength;

end








