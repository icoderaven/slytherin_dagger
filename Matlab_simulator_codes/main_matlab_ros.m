%%%%% Start ROSCORE from terminal before creating the nodes in matlab
% create node that publishes [features actions] to record_simul_bag topic
if ~exist('feat_action_node','var')
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
close all
% position of the target
goal_pt=[40,0,0];
% Initialize snake variables
pitch0=0;
yaw0=0;
drawColor=[0.2 length(state)/66 0.3 ];
LINK_LENGTH=4;
LINK_RADIUS=1.5;
drawType=1;
Tregister=eye(4);
linkStartDraw=0;
[phi0,theta0]=pithyawtoaxisangle(pitch0,yaw0);
% global_state is where the snake actually advances.
global global_state;
global_state = [0,0,0,0,0,0,phi0,theta0];
% state is used for plotting the figures
state = global_state;

% initialize heart variables
heart_coord0 = [10,0,0];
R=[1 0 0; 0 0 -1 ; 0  1 0];
fv = stlread('heart.STL');
coords=R*[fv.vertices(:,1)';fv.vertices(:,2)';fv.vertices(:,3)'];
x=coords(1,:)/10+heart_coord0(1);
y=coords(2,:)/10+heart_coord0(2);
z=coords(3,:)/10+heart_coord0(3);
fv.vertices=[x',y',z'];

% initialize obstacles variables
obstacles = [goal_pt(1) x;goal_pt(2) y;goal_pt(3) z];
% plot initial setup
figure('units','normalized','outerposition',[0 0 1 1])
axis([0 150 -50 50 -50 50]);
grid on
hold on
patch(fv,'FaceColor',[1 0 0],'EdgeColor','none','FaceLighting','gouraud','AmbientStrength', 0.15);
camlight('headlight');
material('dull');
hold on
[~, obstacle] = drawState([10,10,10,0,pi/2,0],[0,0,1],LINK_LENGTH,2*LINK_RADIUS,drawType,Tregister,linkStartDraw);
scatter3(goal_pt(1),goal_pt(2),goal_pt(3),50,'green','fill')
[h, snakePoints] = drawState(state,drawColor,LINK_LENGTH,LINK_RADIUS,drawType,Tregister,linkStartDraw);
hold off
%%
% initialize control variables
over=0;
pitch=0;
yaw=0;
inc=1*pi/180;
maxrange=10*pi/180;
boxsize=10;
steps=50;

% ros variable
start_flag =1;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% publish start recording
if start_flag ==1
    start_msg = rosmatlab.message('std_msgs/Empty', feat_action_node);
    start_pub.publish(start_msg);
    pause(0.1); % to give the bag file time to start recording
end

while over==0 && length(state)<66
    
    
    val=getkey();
    
    %read expert input from keyboard
    [yaw,pitch,state] = get_expert_cmd(state,yaw,pitch,maxrange,inc,val);
    
    if val==32

        % COMPUTE All FEATURES of global_state
        maxdist = 30;
        step=1;
        [feat_array, anchor_pt,normal_vec,head_pt,head_vec] = computeStateFeatures(global_state,LINK_LENGTH,LINK_RADIUS,Tregister,linkStartDraw,obstacles,step,maxdist,goal_pt);
        
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% Communication with ROS
        % 1) Publish the features
        display('[snake_visualize_keyboard_jc]: publishing features')
        vis_msg = rosmatlab.message('std_msgs/Float32MultiArray', feat_action_node);
        vis_msg.setData(feat_array(end:-1:end-1));
        vis_pub.publish(vis_msg);
        % 2) Publish the expert commands
        pause(0.1)
        display('[snake_visualize_keyboard_jc]: sending expert control')
        ctrl_msg = rosmatlab.message('geometry_msgs/Twist', feat_action_node);
        ctrl_msg = construct_ctrl_msg(ctrl_msg,yaw,pitch);
        key_pub.publish(ctrl_msg);
        % 3) The subscriber has probably received the new state update so
        % publish the new global_state
        pause(0.1) % pauses a little to give time for controller to update global_state
        display('[snake_visualize_keyboard_jc]: publishing new global_state')
        state_msg = rosmatlab.message('std_msgs/Float32MultiArray', feat_action_node);
        state_msg.setData(global_state);
        state_pub.publish(state_msg);
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% End of Communication with ROS
        % update state for plotting
        pitch=0;yaw=0;
        state = global_state;
        
    end
    
    if val==113
        display('[snake_visualize_keyboard_jc]: Stopping the loop')
        over=1;
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% ROS: STOP RECORDING
        stop_msg = rosmatlab.message('std_msgs/Empty', feat_action_node);
        stop_pub.publish(stop_msg);
    end
    
    % PLOT FOR EXPLORATION
    plot_currentstate(state,fv,LINK_LENGTH,LINK_RADIUS,Tregister,linkStartDraw,drawType,obstacles,step,maxdist,goal_pt)
    hold off
    
end
h=datestr(clock,30);
% save(h,'log_data')
% end