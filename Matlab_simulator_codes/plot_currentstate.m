function plot_currentstate(state,fv,LINK_LENGTH,LINK_RADIUS,Tregister,linkStartDraw,drawType,obstacles,step,maxdist,goal_pt)
    clf
% compute current features
[~,tmpfeat, anchor_pt,normal_vec,head_pt,head_vec] = computeStateFeatures(state,LINK_LENGTH,LINK_RADIUS,Tregister,linkStartDraw,obstacles,step,maxdist,goal_pt);

drawColor=[0.2 length(state)/66 0.3 ];
hold on
subplot(2,2,1)

patch(fv,'FaceColor',[1 0 0],'EdgeColor','none','FaceLighting','gouraud','AmbientStrength', 0.15);
camlight('headlight');
material('dull');
hold on
%scatter3(x2,y2,z2,'blue','s')
[~, obstacle] = drawState([10,10,10,0,pi/2,0],[0,0,1],LINK_LENGTH,2*LINK_RADIUS,drawType,Tregister,linkStartDraw);

scatter3(goal_pt(1),goal_pt(2),goal_pt(3),150,'green','fill')
[~, snakePoints] = drawState(state,drawColor,LINK_LENGTH,LINK_RADIUS,drawType,Tregister,linkStartDraw);
axis([0 1500 -500 500 -500 500]);
plot_dirlines(anchor_pt,normal_vec,tmpfeat)
plot_dirlines({head_pt},{head_vec},tmpfeat)
title('0,0')
view([0,0])

subplot(2,2,2)

patch(fv,'FaceColor',[1 0 0],'EdgeColor','none','FaceLighting','gouraud','AmbientStrength', 0.15);
camlight('headlight');
material('dull');
hold on
%scatter3(x2,y2,z2,'blue','s')
[~, obstacle] = drawState([10,10,10,0,pi/2,0],[0,0,1],LINK_LENGTH,2*LINK_RADIUS,drawType,Tregister,linkStartDraw);

scatter3(goal_pt(1),goal_pt(2),goal_pt(3),150,'green','fill')
[~, snakePoints] = drawState(state,drawColor,LINK_LENGTH,LINK_RADIUS,drawType,Tregister,linkStartDraw);
axis([0 1500 -500 500 -500 500]);
%     view([90+state(end-1)*180/pi state(end)*180/pi])
title('-180,0')
plot_dirlines(anchor_pt,normal_vec,tmpfeat)
plot_dirlines({head_pt},{head_vec},tmpfeat)
view([-180,0])


subplot(2,2,3)

patch(fv,'FaceColor',[1 0 0],'EdgeColor','none','FaceLighting','gouraud','AmbientStrength', 0.15);
camlight('headlight');
material('dull');
hold on
%scatter3(x2,y2,z2,'blue','s')
[~, obstacle] = drawState([10,10,10,0,pi/2,0],[0,0,1],LINK_LENGTH,2*LINK_RADIUS,drawType,Tregister,linkStartDraw);

scatter3(goal_pt(1),goal_pt(2),goal_pt(3),150,'green','fill')
[~, snakePoints] = drawState(state,drawColor,LINK_LENGTH,LINK_RADIUS,drawType,Tregister,linkStartDraw);
axis([-100 500 -500 500 -500 500]);
%     view([90+state(end-1)*180/pi state(end)*180/pi])
plot_dirlines(anchor_pt,normal_vec,tmpfeat)
plot_dirlines({head_pt},{head_vec},tmpfeat)
title('0,90')
view([0,90])



subplot(2,2,4)
patch(fv,'FaceColor',[1 0 0],'EdgeColor','none','FaceLighting','gouraud','AmbientStrength', 0.15);
camlight('headlight');
material('dull');
hold on
%scatter3(x2,y2,z2,'blue','s')
[~, obstacle] = drawState([10,10,10,0,pi/2,0],[0,0,1],LINK_LENGTH,2*LINK_RADIUS,drawType,Tregister,linkStartDraw);

scatter3(goal_pt(1),goal_pt(2),goal_pt(3),150,'green','fill')
[~, snakePoints] = drawState(state,drawColor,LINK_LENGTH,LINK_RADIUS,drawType,Tregister,linkStartDraw);
axis([0 1500 -500 500 -500 500]);
%view([90+state(end-1)*180/pi state(end)*180/pi])
plot_dirlines(anchor_pt,normal_vec,tmpfeat)
plot_dirlines({head_pt},{head_vec},tmpfeat)
title('-90,90')
view([-90,90])


hold off

end