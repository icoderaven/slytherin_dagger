expert_prob=0;
it_num = 'it4';
if expert_prob <1
    pit_predictor = load(['./predictors/bestl1-',it_num,'s_pitpredictor-20.000000-pit.txt']);
    yaw_predictor = load(['./predictors/bestl1-',it_num,'s_yawpredictor-1.000000-yaw.txt']);
end
%%
close all
% position of the target
goal_pt=[170,+20,0];
% Initialize snake variables
% global_state is where the snake actually advances.
pitch0=0;
yaw0=0;

[phi0,theta0] =pithyawtoaxisangle(pitch0,yaw0);
global global_state;
global_state = [0,0,0,0,0,0,phi0,theta0];
% state is used for plotting the figures
state = global_state;
drawColor=[0.2 length(state)/66 0.3 ];
LINK_LENGTH=15;
LINK_RADIUS=10;
drawType=1;
Tregister=eye(4);
linkStartDraw=0;

% initialize heart variables

R=[1 0 0; 0 0 -1 ; 0  1 0];
scale=2;%everything is defined in cm but stl files are in mm. Hence this scaling.

offset_heart=[50,30,-30];
[ vox_h,fv] = findFilledVoxelsAndRender('heart.STL',R,scale,offset_heart );

offset_obstacle1=[10,70,0];
[ vox_obs1,fv1] = findFilledVoxelsAndRender('obstacle.STL',eye(3),scale*1/2,offset_obstacle1 );

offset_obstacle2=[10,-100,0];
[ vox_obs2,fv2] = findFilledVoxelsAndRender('obstacle.STL',eye(3),scale*1/2,offset_obstacle2 );

%Add more if required
figure('units','normalized','outerposition',[0 0 1 1])
axis([0 500 -200 200 -200 200]);
grid on
hold on
[OUTPUTgrid] = VOXELISE(30,30,30,'heart.STL');
[x,y,z]=ind2sub(size(OUTPUTgrid), find(OUTPUTgrid));
voxels=R*[x';y';z'];
[x,y,z] = scale_shift(voxels,fv);
vox_h=[x',y',z'];

[OUTPUTgrid] = VOXELISE(30,30,30,'obstacle.STL');
[x,y,z]=ind2sub(size(OUTPUTgrid), find(OUTPUTgrid));
voxels=[1,0,0;0,0,-1;0,1,0]*[x';y';z'];
[x,y,z] = scale_shift(voxels,fv1);
vox_obs1=[x',y',z'];

[OUTPUTgrid] = VOXELISE(30,30,30,'obstacle.STL');
[x,y,z]=ind2sub(size(OUTPUTgrid), find(OUTPUTgrid));
voxels=[1,0,0;0,0,-1;0,1,0]*[x';y';z'];
[x,y,z] = scale_shift(voxels,fv2);
vox_obs2=[x',y',z'];


coords_all=[vox_h;vox_obs1;vox_obs2];

obstacles = [goal_pt(1),coords_all(:,1)' ;goal_pt(2),coords_all(:,2)';goal_pt(3),coords_all(:,3)'];


% scatter3(vox_h(:,1),vox_h(:,2),vox_h(:,3),'g')
%scatter3(obstacles(1,:),obstacles(2,:),obstacles(3,:),'m')
hold on
% scatter3(fv.vertices(:,1),fv.vertices(:,2),fv.vertices(:,3),'fill','r')
patch(fv,'FaceColor',[1 0 0],'EdgeColor','none','FaceLighting','gouraud','AmbientStrength', 0.15,'FaceAlpha',0.5);%render heart
patch(fv1,'FaceColor',[1 1 1],'EdgeColor','none','FaceLighting','gouraud','AmbientStrength', 0.15,'FaceAlpha',0.5);%render obstacle1
patch(fv2,'FaceColor',[1 1 0],'EdgeColor','none','FaceLighting','gouraud','AmbientStrength', 0.15,'FaceAlpha',0.5);%render obstacle2 .
camlight('headlight');
material('dull');

scatter3(goal_pt(1),goal_pt(2),goal_pt(3),150,'green','fill')
[h, snakePoints] = drawState(state,drawColor,LINK_LENGTH,LINK_RADIUS,drawType,Tregister,linkStartDraw);
minx = min([snakePoints(:,1);fv.vertices(:,1);fv1.vertices(:,1);fv2.vertices(:,1)]);
maxx = max([snakePoints(:,1);fv.vertices(:,1);fv1.vertices(:,1);fv2.vertices(:,1)]);
miny = min([snakePoints(:,2);fv.vertices(:,2);fv1.vertices(:,2);fv2.vertices(:,2)]);
maxy = max([snakePoints(:,2);fv.vertices(:,2);fv1.vertices(:,2);fv2.vertices(:,2)]);
minz = min([snakePoints(:,3);fv.vertices(:,3);fv1.vertices(:,3);fv2.vertices(:,3)]);
maxz = max([snakePoints(:,3);fv.vertices(:,3);fv1.vertices(:,3);fv2.vertices(:,3)]);

axis([minx,maxx,miny,maxy,minz,maxz])

hold off


% initialize control variables
over=0;
exp_pitch=0;
exp_yaw=0;
inc=1*pi/180;
maxrange=10*pi/180;
boxsize=100;
steps=50;
feat_action_matrix = [];
maxdist = 300;
step=5;
%%
while over==0 && length(state)<66
    
    val=getkey();
    
    %read expert input from keyboard
    [exp_yaw,exp_pitch,state] = get_expert_cmd(state,exp_yaw,exp_pitch,maxrange,inc,val);
    
    if val==32
        % COMPUTE All FEATURES of global_state
        
        [feat_array,tmp_feat,anchor_pt,normal_vec,head_pt,head_vec] = computeStateFeatures(global_state,LINK_LENGTH,LINK_RADIUS,Tregister,linkStartDraw,obstacles,step,maxdist,goal_pt);
        
        
        pred_yaw = exp_yaw;
        pred_pitch = exp_pitch;
        if expert_prob<1 && rand >= expert_prob
            pred_yaw = linear_predictor(feat_array,yaw_predictor)
            pred_pitch = linear_predictor(feat_array,pit_predictor)
        end
        feat_action_matrix = [feat_action_matrix;feat_array(:)',exp_yaw,exp_pitch,pred_yaw,pred_pitch];
        % update global state based on yaw,pitch
        update_myglobalstate_matlab(pred_yaw,pred_pitch)
        % update state for plotting
        exp_pitch=0;exp_yaw=0;
        state = global_state;
        
    end
    
    if val ==107
        keyboard;
    end
    
    if val==113
        display('[snake_visualize_keyboard_jc]: Stopping the loop')
        over=1;
        
    end
    
    if val~=32 && val~=107 && val~=113
        display('press a different key');
    end
    
    % PLOT FOR EXPLORATION
    plot_currentstate(state,fv,fv1,fv2,LINK_LENGTH,LINK_RADIUS,Tregister,linkStartDraw,drawType,obstacles,step,maxdist,goal_pt)
    hold off
    
end
h=datestr(clock,30);
display('Saving file')
save(['it5_data/it5_feat_action_matrix', h],'feat_action_matrix','snakePoints','state');
% save(h,'log_data')
% end