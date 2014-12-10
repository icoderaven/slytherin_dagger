% create node that publishes [features actions] to record_simul_bag topic
if ~exist('feat_action_node','var')
    feat_action_publisher = rosmatlab.node('feature_action_listener');
    publisher = rosmatlab.publisher('record_simul_bag', 'std_msgs/Float32MultiArray', feat_action_publisher);
end

for i=1:size(feat_action_matrix,1)
    msg = rosmatlab.message('std_msgs/Float32MultiArray', feat_action_publisher);
    msg.setData(feat_matrix(i,:));
    publisher.publish(msg);
end