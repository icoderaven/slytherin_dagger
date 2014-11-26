function [X, P] = predictSteering(X, P, phi, theta, linkLengthsTurning, steerAngularUncertaintyPhi, steerAngularUncertaintyTheta)
% function [X, P] = predictSteering(X, P, phi, theta, linkLengthsTurning, steerAngularUncertaintyPhi, steerAngularUncertaintyTheta)

X(end-1) = phi/linkLengthsTurning;
X(end) = theta;
P(end-1,end-1) = P(end-1,end-1) + steerAngularUncertaintyPhi;
P(end,end) = P(end,end) + steerAngularUncertaintyTheta;

