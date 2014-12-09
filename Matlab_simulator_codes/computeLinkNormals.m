function [normal_vec,anchor_pt,skip_index] = computeLinkNormals(T, linkLength, radius)

% modified the drawLink function to compute the normals to the links

% loop through in polar coords and compute the normals to the polygons
numThetas = 18;
counter = 1;
skip_index = 5;
normal_vec = zeros([length(1:skip_index:numThetas),3]);
anchor_pt = zeros([length(1:skip_index:numThetas),3]);

for i = 1:skip_index:numThetas
    theta = 2*pi*(i-1)/numThetas;
    P = [0,0,-linkLength,-linkLength;radius*cos(theta),radius*cos(theta+2*pi/numThetas),radius*cos(theta+2*pi/numThetas),radius*cos(theta);radius*sin(theta),radius*sin(theta+2*pi/numThetas),radius*sin(theta+2*pi/numThetas),radius*sin(theta);1,1,1,1];
    P = T*P;
    p1 = [P(1,1),P(2,1),P(3,1)];
    p2 = [P(1,2),P(2,2),P(3,2)];
    p3 = [P(1,3),P(2,3),P(3,3)];
    anchor_pt(counter,:) = p1;
    normal_vec(counter,:) = cross(p2-p1,p3-p1);
    counter = counter + 1;
end


end


