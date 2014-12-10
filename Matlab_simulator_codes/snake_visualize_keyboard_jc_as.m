%%
close all
% position of the target
goal_pt=[40,0,0];

pitch0=0;
yaw0=0;
[phi0,theta0]=pithyawtoaxisangle(pitch0,yaw0);

% glbal_state is where the snake actually advances. 
global global_state;
global_state = [0,0,0,0,0,0,phi0,theta0];
% state is used for expert exploration, once global_state is updated,
% it is updated with its value.
state = global_state;

drawColor=[0.2 length(state)/66 0.3 ];
LINK_LENGTH=4;
LINK_RADIUS=1.5;
drawType=1;
Tregister=eye(4);
linkStartDraw=0;

%Voxelise the STL: This will be used to compute distance features. For
%visualition and navigation, better to use an stl 3D renderer
R=[1 0 0; 0 0 -1 ; 0  1 0];
fv = stlread('heart.STL');
%patch(fv,'FaceColor',       [1 0 0], 'EdgeColor',       'none',    'FaceLighting',    'gouraud',      'AmbientStrength', 0.15);
%[OUTPUTgrid] = VOXELISE(20,20,20,'heart.STL','xyz');
coords=R*[fv.vertices(:,1)';fv.vertices(:,2)';fv.vertices(:,3)'];
x=coords(1,:)/10+10;
y=coords(2,:)/10;%+5;
z=coords(3,:)/10;%-10;
fv.vertices=[x',y',z'];

%fv2 = stlread('sample.STL');
%coords2=R*[fv2.vertices(:,1)';fv2.vertices(:,2)';fv2.vertices(:,3)'];
%x2=coords2(1,:)/10;%+10;
%y2=coords2(2,:)/10;%+5;
%z2=coords2(3,:)/10;%-10;
%fv2.vertices=[x2',y2',z2'];

%[OUTPUTgrid2] = VOXELISE(20,20,20,'sample.stl','xyz');
%[x,y,z]=ind2sub(size(OUTPUTgrid), find(OUTPUTgrid));
%[x2,y2,z2]=ind2sub(size(OUTPUTgrid2), find(OUTPUTgrid2));
%voxels2=R*[x2';y2';z2']/5;
%x2=voxels2(1,:)+20;
%y2=voxels2(2,:)+20;
%z2=voxels2(3,:);

%obstacles = [x, x2;y,y2;z,z2];
figure('units','normalized','outerposition',[0 0 1 1])
axis([0 150 -50 50 -50 50]);
grid on
hold on
%scatter3(x,y,z,'red','s');
patch(fv,'FaceColor',[1 0 0],'EdgeColor','none','FaceLighting','gouraud','AmbientStrength', 0.15);
hold on
%scatter3(x2,y2,z2,'blue','s')
[~, obstacle] = drawState([10,10,10,0,pi/2,0],[0,0,1],LINK_LENGTH,2*LINK_RADIUS,drawType,Tregister,linkStartDraw);
%patch(fv2,'FaceColor',[0 0 1],'EdgeColor','none','FaceLighting','gouraud','AmbientStrength', 0.15);
scatter3(goal_pt(1),goal_pt(2),goal_pt(3),150,'green','fill')
[h, snakePoints] = drawState(state,drawColor,LINK_LENGTH,LINK_RADIUS,drawType,Tregister,linkStartDraw);
%%
spx=snakePoints(:,1);spy=snakePoints(:,2);spz=snakePoints(:,3);

%%%%%Xadd, Yadd and Z add are just a few points on the circumference of the
%%%%%cylinder. We need this along with snake points to find the bounding
%%%%%box
xadd=[spx(1);spx(1);spx(1);spx(1);spx(2);spx(2);spx(2);spx(2)];
yadd=[spy(1)+LINK_RADIUS;spy(1)-LINK_RADIUS;spy(1);spy(1);spy(2)+LINK_RADIUS;spy(2)-LINK_RADIUS;spy(2);spy(2)];
zadd=[spz(1);spz(1);spz(1)-LINK_RADIUS;spz(1)+LINK_RADIUS;spz(1);spz(2);spz(2)-LINK_RADIUS;spz(2)+LINK_RADIUS];
%%
over=0;
pitch=0;
yaw=0;
inc=1*pi/180;
maxrange=10*pi/180;
boxsize=10;
steps=50;
count=0;
start_flag =1;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% publish start recording

while over==0 && length(state)<66 
    
    
%     [voxel_mat,filled_voxels,flag,validpoints]=findvoxelsinbox(voxels,snakePoints(end,:)',boxsize,steps);
    val=getkey();
    
    %read from keyboard
    [yaw,pitch,state] = get_expert_cmd(state,yaw,pitch,maxrange,inc,val);
    
    if val==32
        %if flag==1
        count=count+1;
%         log_data{count}=[filled_voxels,pitch,yaw];
        
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% Publish features
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% and actions
                % COMPUTE DISTANCE FEATURES
        maxdist = 30;
        step=1;
        [feat_array, anchor_pt,normal_vec,head_pt,head_vec] = computeStateFeatures(global_state,LINK_LENGTH,LINK_RADIUS,Tregister,linkStartDraw,obstacles,step,maxdist);
        
        pause(0.1)
        pitch=0;yaw=0;
        %         global_state
        %state_msg = rosmatlab.message('std_msgs/Float32MultiArray', feat_action_node);
        %state_msg.setData(global_state);
        pause(0.5);
        %state_pub.publish(state_msg);
        
        state = global_state;
        %         state=adderror(state,1);
        %         state=[state,0,0];
        [feat_array, anchor_pt,normal_vec,head_pt,head_vec] = computeStateFeatures(global_state,LINK_LENGTH,LINK_RADIUS,Tregister,linkStartDraw,obstacles,step,maxdist);

    end
    
    clf
    
    
    drawColor=[0.2 length(state)/66 0.3 ];
    hold on
    subplot(2,2,1)
    
    scatter3(x,y,z,'red','fill','s');
    hold on
    scatter3(x2,y2,z2,'blue','s')

    scatter3(goal_pt(1),goal_pt(2),goal_pt(3),150,'green','fill')
    [~, snakePoints] = drawState(state,drawColor,LINK_LENGTH,LINK_RADIUS,drawType,Tregister,linkStartDraw);
    % Distance to goal
    distanceToGoal=norm(snakePoints(end,:)-goal_pt);
    cur_vec=snakePoints(end,:)-snakePoints(end-1,:);
    req_vec=goal_pt-snakePoints(end,:);
    angleToGoal= acos(cur_vec*req_vec'/(norm(cur_vec)*norm(req_vec)));
    

    
    %
    %%%%%the next two lines will ifnd the bounding box and plot it. We
    %%%%%include the points on circumference of cylinder to the snake
    %%%%%points and then plot them. The variable volume returns the volume
    %%%%%directly that can be used as a feature. The edgelength can also be
    %%%%%used as a feature. 
    [rotmat,cornerpoints,volume,surface,edgelength] = minboundbox([snakePoints(:,1);xadd],[snakePoints(:,2);yadd],[snakePoints(:,3);zadd]);
    plotminbox(cornerpoints,'red')

%     P1 = snakePoints(end,:);
%     P2 = snakePoints(end,:)+100*(snakePoints(end,:)-snakePoints(end-1,:))/norm((snakePoints(end,:)-snakePoints(end-1,:)));
%     pts = [P1; P2];
%     line(pts(:,1), pts(:,2), pts(:,3))
    axis([0 150 -50 50 -50 50]);
    plot_dirlines(anchor_pt,normal_vec,feat_array(1:end-1))
    plot_dirlines({head_pt},{head_vec},feat_array(end))
    title('0,0')
    view([0,0])
    
    subplot(2,2,2)
    
    scatter3(x,y,z,'red','fill','s');
    hold on
    scatter3(x2,y2,z2,'blue','s')
    scatter3(goal_pt(1),goal_pt(2),goal_pt(3),150,'green','fill')
    [~, snakePoints] = drawState(state,drawColor,LINK_LENGTH,LINK_RADIUS,drawType,Tregister,linkStartDraw);
%     line(pts(:,1), pts(:,2), pts(:,3))
    axis([0 150 -50 50 -50 50]);
    %     view([90+state(end-1)*180/pi state(end)*180/pi])
    title('-180,0')
    plot_dirlines(anchor_pt,normal_vec,feat_array(1:end-1))
    plot_dirlines({head_pt},{head_vec},feat_array(end))
    view([-180,0])
    
    
    subplot(2,2,3)
    
    scatter3(x,y,z,'red','fill','s');
    hold on
    scatter3(x2,y2,z2,'blue','s')
    scatter3(goal_pt(1),goal_pt(2),goal_pt(3),150,'green','fill')
    [~, snakePoints] = drawState(state,drawColor,LINK_LENGTH,LINK_RADIUS,drawType,Tregister,linkStartDraw);
%     line(pts(:,1), pts(:,2), pts(:,3))
    axis([-10 50 -50 50 -50 50]);
    %     view([90+state(end-1)*180/pi state(end)*180/pi])
    plot_dirlines(anchor_pt,normal_vec,feat_array(1:end-1))
    plot_dirlines({head_pt},{head_vec},feat_array(end))
    title('0,90')
    view([0,90])
    
    
    
    subplot(2,2,4)
    scatter3(x,y,z,'red','fill','s');
    hold on
    scatter3(x2,y2,z2,'blue','s')
    scatter3(goal_pt(1),goal_pt(2),goal_pt(3),150,'green','fill')
    [~, snakePoints] = drawState(state,drawColor,LINK_LENGTH,LINK_RADIUS,drawType,Tregister,linkStartDraw);
%     line(pts(:,1), pts(:,2), pts(:,3))
    axis([0 150 -50 50 -50 50]);
    %view([90+state(end-1)*180/pi state(end)*180/pi])
    plot_dirlines(anchor_pt,normal_vec,feat_array(1:end-1))
    plot_dirlines({head_pt},{head_vec},feat_array(end))
    title('-90,90')
    view([-90,90])
    
    
    hold off
    
    
    
end
h=datestr(clock,30);
% save(h,'log_data')
% end