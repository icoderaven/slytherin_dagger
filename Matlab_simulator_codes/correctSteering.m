function [X, P] = correctSteering(X, P, trackerPose, measurementAngularUncertainty, measurementSpatialUncertainty, linkLength)
% function [X, P] = correctSteering(X, P, trackerPose, measurementAngularUncertainty, measurementSpatialUncertainty, linkLength)

% get the tip transformation matrix
Ttip = stateToTransformationMatrix(X, linkLength);

% extract the expected pose angles
rz_state  = atan2(Ttip(2,1), Ttip(1,1));
ry_state  = atan2(-Ttip(3,1), sqrt(Ttip(3,2)^2+Ttip(3,3)^2));
rz_sensed = atan2(trackerPose(2,1), trackerPose(1,1));
ry_sensed = atan2(-trackerPose(3,1), sqrt(trackerPose(3,2)^2+trackerPose(3,3)^2));

% compute the measurement
res = trackerPose(1:3,4) - Ttip(1:3,4);
res(4,1) = rz_sensed - rz_state;
while (res(4,1) < -pi), res(4,1) = res(4,1) + 2*pi; end;
while (res(4,1) >  pi), res(4,1) = res(4,1) - 2*pi; end;
res(5,1) = ry_sensed - ry_state;
while (res(5,1) < -pi), res(5,1) = res(5,1) + 2*pi; end;
while (res(5,1) >  pi), res(5,1) = res(5,1) - 2*pi; end;

% compute the Jacobian matrix
delta = 1e-5;
H = [];
for i = 1:length(X),
    Xdelta = X;
    Xdelta(i) = Xdelta(i) + delta;
    TtipDelta = stateToTransformationMatrix(Xdelta, linkLength);
    rz_state_delta  = atan2(TtipDelta(2,1), TtipDelta(1,1));% arun changed Ttip to TtipDelta
    ry_state_delta  = atan2(-TtipDelta(3,1), sqrt(TtipDelta(3,2)^2+TtipDelta(3,3)^2));% arun changed Ttip to TtipDelta
    hDiff = [TtipDelta(1:3,4) - Ttip(1:3,4); rz_state_delta - rz_state; ry_state_delta - ry_state];
    while (hDiff(4,1) < -pi), hDiff(4,1) = hDiff(4,1) + 2*pi; end;
    while (hDiff(4,1) >  pi), hDiff(4,1) = hDiff(4,1) - 2*pi; end;
    while (hDiff(5,1) < -pi), hDiff(5,1) = hDiff(5,1) + 2*pi; end;
    while (hDiff(5,1) >  pi), hDiff(5,1) = hDiff(5,1) - 2*pi; end;
    H(:,i) = hDiff./delta;
end

% perform the update
R = [measurementSpatialUncertainty.*eye(3), zeros(3,2); zeros(2,3), measurementAngularUncertainty.*eye(2,2)];
K = P*H'*inv(H*P*H' + R);
X = X + K*res;
P = P - K*H*P;
