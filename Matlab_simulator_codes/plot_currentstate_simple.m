function plot_currentstate_simple(state,fv,fv1,fv2,LINK_LENGTH,LINK_RADIUS,Tregister,linkStartDraw,drawType,obstacles,step,maxdist,goal_pt)
    clf

drawColor=[0.2 length(state)/66 0.3 ];
hold on
%subplot(2,2,1)
 patch(fv,'FaceColor',[1 0 0],'EdgeColor','none','FaceLighting','gouraud','AmbientStrength', 0.15,'FaceAlpha',0.1);
 patch(fv1,'FaceColor',[1 1 0],'EdgeColor','none','FaceLighting','gouraud','AmbientStrength', 0.15);
 patch(fv2,'FaceColor',[1 1 0],'EdgeColor','none','FaceLighting','gouraud','AmbientStrength', 0.15);
hold on
%scatter3(obstacles(1,:),obstacles(2,:),obstacles(3,:),'m')

scatter3(goal_pt(1),goal_pt(2),goal_pt(3),150,'green','fill')
[~, snakePoints] = drawState(state,drawColor,LINK_LENGTH,LINK_RADIUS,drawType,Tregister,linkStartDraw);
campos([snakePoints(1,1),snakePoints(1,2),snakePoints(1,3)+200])
camlight('headlight');
material('dull');
hold on
%scatter3(x2,y2,z2,'blue','s')
%[~, obstacle] = drawState([10,10,10,0,pi/2,0],[0,0,1],LINK_LENGTH,2*LINK_RADIUS,drawType,Tregister,linkStartDraw);

minx = min([snakePoints(:,1);fv.vertices(:,1);fv1.vertices(:,1);fv2.vertices(:,1)]);
maxx = max([snakePoints(:,1);fv.vertices(:,1);fv1.vertices(:,1);fv2.vertices(:,1)]);
miny = min([snakePoints(:,2);fv.vertices(:,2);fv1.vertices(:,2);fv2.vertices(:,2)]);
maxy = max([snakePoints(:,2);fv.vertices(:,2);fv1.vertices(:,2);fv2.vertices(:,2)]);
minz = min([snakePoints(:,3);fv.vertices(:,3);fv1.vertices(:,3);fv2.vertices(:,3)]);
maxz = max([snakePoints(:,3);fv.vertices(:,3);fv1.vertices(:,3);fv2.vertices(:,3)]);

% axis([0 1500 -500 500 -500 500]);
axis([minx,maxx,miny,maxy,minz,maxz])
%plot_dirlines(anchor_pt,normal_vec,tmpfeat)
%plot_dirlines({head_pt},{head_vec},head_feat)
% campos([113,23,16])
% campos([goal_pt(1),goal_pt(2),goal_pt(3)+100])
title('0,0')
% view([0,0])
hold off
end