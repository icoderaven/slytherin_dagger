    
function [h] = drawLink(T, linkLength, radius, drawColor, mode)

h = [];

% check for simple mode
if (mode == 0)
    display('arun');
    point1 = T*[0; 0; 0; 1];
    point2 = T*[-linkLength; 0; 0; 1];   
    h = [h, plot3([point1(1) point2(1)],[point1(2) point2(2)],[point1(3) point2(3)],'Color',drawColor)];    
    h = [h, plot3(point1(1),point1(2),point1(3),'.','Color',drawColor)];    
else
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
    h = [h, patch(xs,ys,zs,drawColor)];
    set(h(end),'FaceColor',drawColor,'EdgeColor',0.8.*drawColor,'EdgeLighting','phong','FaceLighting','phong','EdgeAlpha',1.0,'FaceAlpha',1.0);
    set(h(end),'SpecularStrength',0.4,'AmbientStrength',0.3,'DiffuseStrength',0.9,'SpecularExponent',100);
end
    
    
