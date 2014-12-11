%%
close all
% position of the target
goal_pt=[180,0,0];
% Initialize snake variables
% global_state is where the snake actually advances.
pitch0=0;
yaw0=0;

[phi0,theta0]=pithyawtoaxisangle(pitch0,yaw0);
global global_state;
global_state = [0,0,0,0,0,0,phi0,theta0];
% state is used for plotting the figures
state = global_state;
drawColor=[0.2 length(state)/66 0.3 ];
LINK_LENGTH=15;
LINK_RADIUS=15;
drawType=1;
Tregister=eye(4); 
linkStartDraw=0;



% initialize heart variables

R=[1 0 0; 0 0 -1 ; 0  1 0];
scale=2;%everything is defined in cm but stl files are in mm. Hence this scaling.

offset_heart=[30,30,-20];
[ vox_h,fv] = findFilledVoxelsAndRender('heart.STL',R,scale,offset_heart );

offset_obstacle1=[10,50,0];
[ vox_obs1,fv1] = findFilledVoxelsAndRender('obstacle.STL',eye(3),scale*1/2,offset_obstacle1 );

offset_obstacle2=[10,-100,0];
[ vox_obs2,fv2] = findFilledVoxelsAndRender('obstacle.STL',eye(3),scale*1/2,offset_obstacle2 );

%Add more if required
%offset_obstacle3=[-10,-10,10];
%[ vox_obs3,fv3] = findFilledVoxelsAndRender('obstacle.STL',R,scale,offset_obstacle3 );
figure('units','normalized','outerposition',[0 0 1 1])
axis([0 500 -200 200 -200 200]);
grid on
hold on
%scatter3(obstacles(1,:),obstacles(2,:),obstacles(3,:),'fill','k')
[OUTPUTgrid] = VOXELISE(30,30,30,'heart.STL');
[x,y,z]=ind2sub(size(OUTPUTgrid), find(OUTPUTgrid));
voxels=R*[x';y';z'];
% x=voxels(1,:)*4+60;
% y=voxels(2,:)*3+40;
% z=voxels(3,:)*6-20;
[x,y,z] = scale_shift(voxels,fv);
vox_h=[x',y',z'];


[OUTPUTgrid] = VOXELISE(30,30,30,'obstacle.STL');
[x,y,z]=ind2sub(size(OUTPUTgrid), find(OUTPUTgrid));
voxels=[1,0,0;0,0,-1;0,1,0]*[x';y';z'];
% x=voxels(1,:)*10+5;
% y=voxels(2,:)*3+110;
% z=voxels(3,:)*10-10;
[x,y,z] = scale_shift(voxels,fv1);
vox_obs1=[x',y',z'];



[OUTPUTgrid] = VOXELISE(30,30,30,'obstacle.STL');
[x,y,z]=ind2sub(size(OUTPUTgrid), find(OUTPUTgrid));
voxels=[1,0,0;0,0,-1;0,1,0]*[x';y';z'];
% x=voxels(1,:)*10+5;
% y=voxels(2,:)*3-45;
% z=voxels(3,:)*10-10;
[x,y,z] = scale_shift(voxels,fv2);
vox_obs2=[x',y',z'];


coords_all=[vox_h;vox_obs1;vox_obs2];

obstacles = [goal_pt(1),coords_all(:,1)' ;goal_pt(2),coords_all(:,2)';goal_pt(3),coords_all(:,3)'];



% scatter3(vox_h(:,1),vox_h(:,2),vox_h(:,3),'g')
scatter3(obstacles(1,:),obstacles(2,:),obstacles(3,:),'m')
hold on
% scatter3(fv.vertices(:,1),fv.vertices(:,2),fv.vertices(:,3),'fill','r')
patch(fv,'FaceColor',[1 0 0],'EdgeColor','none','FaceLighting','gouraud','AmbientStrength', 0.15);%render heart
patch(fv1,'FaceColor',[1 1 1],'EdgeColor','none','FaceLighting','gouraud','AmbientStrength', 0.15);%render obstacle1
patch(fv2,'FaceColor',[1 1 0],'EdgeColor','none','FaceLighting','gouraud','AmbientStrength', 0.15);%render obstacle2 . 
camlight('headlight');
material('dull');

scatter3(goal_pt(1),goal_pt(2),goal_pt(3),150,'green','fill')
[h, snakePoints] = drawState(state,drawColor,LINK_LENGTH,LINK_RADIUS,drawType,Tregister,linkStartDraw);
hold off

%%
% initialize control variables
over=0;
pitch=0;
yaw=0;
inc=1*pi/180;
maxrange=10*pi/180;
boxsize=100;
steps=50;
expert_prob=1;
while over==0 && length(state)<66
    
    val=getkey();
    
    %read expert input from keyboard
    [yaw,pitch,state] = get_expert_cmd(state,yaw,pitch,maxrange,inc,val);

    if val==32
        
        % COMPUTE All FEATURES of global_state
        maxdist = 300;
        step=5;
        [feat_array,tmp_feat,anchor_pt,normal_vec,head_pt,head_vec] = computeStateFeatures(global_state,LINK_LENGTH,LINK_RADIUS,Tregister,linkStartDraw,obstacles,step,maxdist,goal_pt);
        
        if expert_prob==1
            pred_yaw = yaw;
            pred_pitch = pitch;
        end
        % update global state based on yaw,pitch
        update_myglobalstate_matlab(pred_yaw,pred_pitch)
        global_state
        % update state for plotting
        pitch=0;yaw=0;
        state = global_state;
        
    end
    
    if val ==107
        keyboard;
    end
    
    if val==113
        display('[snake_visualize_keyboard_jc]: Stopping the loop')
        over=1;
        
    end
    
    % PLOT FOR EXPLORATION
    plot_currentstate(state,fv,fv1,fv2,LINK_LENGTH,LINK_RADIUS,Tregister,linkStartDraw,drawType,obstacles,step,maxdist,goal_pt)
    hold off
    
end
h=datestr(clock,30);
% save(h,'log_data')
% end