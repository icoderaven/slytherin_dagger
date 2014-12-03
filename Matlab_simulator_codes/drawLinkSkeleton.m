
function [h] = drawLinkSkeleton(T, linkLength, radius)

h = [];

% loop through in polar coords and draw all the polygons
numPhis = 5;
numThetas = 18;
xs = []; ys = []; zs = [];
for i = 1:numThetas,
    theta = 2*pi*(i-1)/numThetas;
    newPoints = [0,0,-linkLength,-linkLength;radius*cos(theta),radius*cos(theta+2*pi/numThetas),radius*cos(theta+2*pi/numThetas),radius*cos(theta);radius*sin(theta),radius*sin(theta+2*pi/numThetas),radius*sin(theta+2*pi/numThetas),radius*sin(theta); 1 1 1 1];
    newPoints = T*newPoints;
    xs = [xs,newPoints(1,:)'];
    ys = [ys,newPoints(2,:)'];
    zs = [zs,newPoints(3,:)'];  
    for j = 1:numPhis,
        phi = pi*(j-1)/(2*numPhis);
        newPoints = [radius*sin(phi),radius*sin(phi),radius*sin(phi+pi/(2*numPhis)),radius*sin(phi+pi/(2*numPhis));radius*cos(theta)*cos(phi),radius*cos(theta+2*pi/numThetas)*cos(phi),radius*cos(theta+2*pi/numThetas)*cos(phi+pi/(2*numPhis)),radius*cos(theta)*cos(phi+pi/(2*numPhis));radius*sin(theta)*cos(phi),radius*sin(theta+2*pi/numThetas)*cos(phi),radius*sin(theta+2*pi/numThetas)*cos(phi+pi/(2*numPhis)),radius*sin(theta)*cos(phi+pi/(2*numPhis)); 1 1 1 1];
        newPoints = T*newPoints;
        xs = [xs,newPoints(1,:)'];
        ys = [ys,newPoints(2,:)'];
        zs = [zs,newPoints(3,:)'];
    end
end
h = [h, patch(xs,ys,zs,[0 0 0])];
set(h(end),'FaceColor',[0.95 0.95 0.95],'EdgeColor',[0.4 0.4 0.4],'EdgeLighting','phong','FaceLighting','phong','EdgeAlpha',1.0,'FaceAlpha',1.0);
axis equal;
axis([-20 20 -20 20 -20 20]);
    
    
