%launch a roscore master
% setenv('ROS_MASTER_URI','http://bird:11311/')
% setenv('ROS_HOSTNAME','bird')
% if ~exist('roscore','var')
%     display('Starting ROS')
%     roscore = rosmatlab.roscore(11311);
% end
% create node that publishes [features actions] to record_simul_bag topic
if ~ exist('feat_action_node','var')
    display('Creating Node')
    feat_action_node = rosmatlab.node('feature_action_node');%, roscore.RosMasterUri);
    
    % publishers
    %     rec_pub = rosmatlab.publisher('sim_rec', 'std_msgs/Float32MultiArray', feat_action_node);
    
    start_pub  = rosmatlab.publisher('key_start', 'std_msgs/Empty', feat_action_node);
    
    stop_pub = rosmatlab.publisher('key_stop', 'std_msgs/Empty', feat_action_node);
    
    key_pub = rosmatlab.publisher('key_vel', 'geometry_msgs/Twist', feat_action_node);
    
    vis_pub = rosmatlab.publisher('vis_features', 'std_msgs/Float32MultiArray', feat_action_node);
    
    state_pub = rosmatlab.publisher('pose_info', 'std_msgs/Float32MultiArray', feat_action_node);
    % subscriber
    vel_sub = rosmatlab.subscriber('sim_cmd_vel','geometry_msgs/Twist',10,feat_action_node); % subscribes to a Twist message
    vel_sub.setOnNewMessageListeners({@update_myglobalstate})
end
%%
%hold on
close all
% clear
clc
P3=[40,0,0];

pitch0=0;
yaw0=0;
[phi0,theta0]=pithyawtoaxisangle(pitch0,yaw0);

% true_state is where the snake actually advances.
global global_state;
global_state = [0,0,0,0,0,0,phi0,theta0];
% state is used for expert exploration.
state = global_state;

drawColor=[0.2 length(state)/66 0.3 ];%drawColor=drawColor/norm(drawColor);
LINK_LENGTH=4;
LINK_RADIUS=1.5;
drawType=1;
Tregister=eye(4);
linkStartDraw=0;
%axis([0 150 -50 50 -50 50]);
%Voxelise the STL:
R=[1 0 0; 0 0 -1 ; 0  1 0];

[OUTPUTgrid] = VOXELISE(20,20,20,'heart.STL','xyz');
[x,y,z]=ind2sub(size(OUTPUTgrid), find(OUTPUTgrid));
voxels=R*[x';y';z'];
x=voxels(1,:)+10;
y=voxels(2,:)+5;
z=voxels(3,:)-10;
figure('units','normalized','outerposition',[0 0 1 1])
axis([0 150 -50 50 -50 50]);
grid on
hold on
subplot(1,2,1)

scatter3(x,y,z,'red','s');
hold on
scatter3(P3(1),P3(2),P3(3),150,'green','fill')

[h, snakePoints] = drawState(state,drawColor,LINK_LENGTH,LINK_RADIUS,drawType,Tregister,linkStartDraw);
subplot(1,2,2)
scatter3(x,y,z);
[h, snakePoints] = drawState(state,drawColor,LINK_LENGTH,LINK_RADIUS,drawType,Tregister,linkStartDraw);
view([90+state(end-1) state(end)])
hold off

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
if start_flag ==1
    start_msg = rosmatlab.message('std_msgs/Empty', feat_action_node);
    start_pub.publish(start_msg);
end

while over==0 && length(state)<=66
    
    
    [voxel_mat,filled_voxels,flag,validpoints]=findvoxelsinbox(voxels,snakePoints(end,:)',boxsize,steps);
%     voxel_mat(:)
    val=getkey();
    %read from keyboard
    [yaw,pitch,state] = get_expert_cmd(state,yaw,pitch,maxrange,inc,val);
    
    if val==32
        if flag==1
            count=count+1;
            log_data{count}=[filled_voxels,pitch,yaw];
            
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% Publish features
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% and actions
            vis_msg = rosmatlab.message('std_msgs/Float32MultiArray', feat_action_node);
            vis_msg.setData(filled_voxels);
            vis_pub.publish(vis_msg);
            
            pause(0.1)
            ctrl_msg = rosmatlab.message('geometry_msgs/Twist', feat_action_node);
            ctrl_msg = construct_ctrl_msg(ctrl_msg,yaw,pitch);
            key_pub.publish(ctrl_msg);
        end
        pause(0.1)
        %%%%% this part is done when subscriber receives control;
        pitch=0;yaw=0;
        %         global_state
        state_msg = rosmatlab.message('std_msgs/Float32MultiArray', feat_action_node);
        state_msg.setData(global_state);
        pause(0.5);
        state_pub.publish(state_msg);
        
        state = global_state;
        %         state=adderror(state,1);
        %         state=[state,0,0];
        
    end
    if val==113
        over=1;
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% STOP RECORDING
        stop_msg = rosmatlab.message('std_msgs/Empty', feat_action_node);
        stop_pub.publish(stop_msg);
    end
    clf
    
    
    drawColor=[0.2 length(state)/66 0.3 ];
    hold on
    subplot(2,2,1)
    
    scatter3(x,y,z,'red','fill','s');
    hold on
    if length(validpoints)>1
        scatter3(validpoints(1,:)+10,validpoints(2,:)+5,validpoints(3,:)-10,'black','fill','s');
    end
    
    scatter3(P3(1),P3(2),P3(3),150,'green','fill')
    [~, snakePoints] = drawState(state,drawColor,LINK_LENGTH,LINK_RADIUS,drawType,Tregister,linkStartDraw);
    P1 = snakePoints(end,:);
    P2 = snakePoints(end,:)+100*(snakePoints(end,:)-snakePoints(end-1,:))/norm((snakePoints(end,:)-snakePoints(end-1,:)));
    pts = [P1; P2];
    line(pts(:,1), pts(:,2), pts(:,3))
    axis([0 150 -50 50 -50 50]);
    title('0,0')
    view([0,0])
    
    subplot(2,2,2)
    
    scatter3(x,y,z,'red','fill','s');
    hold on
    
    if length(validpoints)>1
        scatter3(validpoints(1,:)+10,validpoints(2,:)+5,validpoints(3,:)-10,'black','fill','s');
    end
    scatter3(P3(1),P3(2),P3(3),150,'green','fill')
    [~, snakePoints] = drawState(state,drawColor,LINK_LENGTH,LINK_RADIUS,drawType,Tregister,linkStartDraw);
    line(pts(:,1), pts(:,2), pts(:,3))
    axis([0 150 -50 50 -50 50]);
    %     view([90+state(end-1)*180/pi state(end)*180/pi])
    title('-180,0')
    view([-180,0])
    
    
    subplot(2,2,3)
    
    scatter3(x,y,z,'red','fill','s');
    hold on
    if length(validpoints)>1
        scatter3(validpoints(1,:)+10,validpoints(2,:)+5,validpoints(3,:)-10,'black','fill','s');
    end
    scatter3(P3(1),P3(2),P3(3),150,'green','fill')
    [~, snakePoints] = drawState(state,drawColor,LINK_LENGTH,LINK_RADIUS,drawType,Tregister,linkStartDraw);
    line(pts(:,1), pts(:,2), pts(:,3))
    axis([0 150 -50 50 -50 50]);
    %     view([90+state(end-1)*180/pi state(end)*180/pi])
    title('0,90')
    view([0,90])
    
    
    
    subplot(2,2,4)
    
    scatter3(x,y,z,'red','fill','s');
    hold on
    if length(validpoints)>1
        scatter3(validpoints(1,:)+10,validpoints(2,:)+5,validpoints(3,:)-10,'black','fill','s');
    end
    scatter3(P3(1),P3(2),P3(3),150,'green','fill')
    [~, snakePoints] = drawState(state,drawColor,LINK_LENGTH,LINK_RADIUS,drawType,Tregister,linkStartDraw);
    line(pts(:,1), pts(:,2), pts(:,3))
    axis([0 150 -50 50 -50 50]);
    %view([90+state(end-1)*180/pi state(end)*180/pi])
    title('-90,90')
    view([-90,90])
    
    
    hold off
    
    
    
end
h=datestr(clock,30);
save(h,'log_data')
% end