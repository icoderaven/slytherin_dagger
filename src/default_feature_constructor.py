#!/usr/bin/env python  
import math
import numpy as np

import tf
from tf.transformations import euler_from_quaternion

def compute_yaw(q):
	rpy = euler_from_quaternion([q[0],q[1],q[2],q[3]])
	return rpy[2]

def compute_goal_dir(pos,q,goal):
	dx = goal[0]-pos[0]
	dy = goal[1]-pos[1]
	theta = math.atan2(dy,dx) #compute angle between current location and goal
	#theta now contain the goal direction if quadrotor is facing x direction
	#add offset to account for current yaw orientation
	theta -= compute_yaw(q)
	if theta > math.pi:
		theta -= 2.0*math.pi
	if theta < -math.pi:
		theta += 2.0*math.pi
	return theta

def low_pass_filter(last_filter,y,dt,K):
	alpha = dt/(dt+K)
	return last_filter + alpha*y
	

#---------------------------------------------------------------------------
#construct feature vector from visual data and position, quaternion and goal
#---------------------------------------------------------------------------
def construct(vis_feat,prv_ctrl_feat):#,pos,q,goal):
	#compute direction to goal from current position, quaternion and goal
	#theta = compute_goal_dir(pos,q,goal)		
	tmp_feat = np.append(vis_feat, prv_ctrl_feat)
	return np.append(tmp_feat, [1])

def construct_weighted(vis_feat,prv_ctrl_feat, d_yaw, y_velocity):
    # feat_array = [visual features, yaw offset from requested value,
    #               current y velocity, vector of previous commands, offset = 1] 
    feat_array = np.append(vis_feat, d_yaw)
    feat_array = np.append(feat_array, y_velocity) 
    feat_array = np.append(feat_array, prv_ctrl_feat)
    feat_array = np.append(feat_array, [1])
    
	# corresponding weights - change these in train.py if you change the feature vector!
    #feat_weights = np.append(np.ones(vis_feat.size)*4.0/(1.0*vis_feat.size), [1])
    #feat_weights = np.append(feat_weights, [1]) 
    #prev_cmd_weights = np.ones(prv_ctrl_feat.size)
    #for i in range(prv_ctrl_feat.size):
	#   prev_cmd_weights[i] = pow(2.0,-i)
    #feat_weights = np.append(feat_weights, prev_cmd_weights)
    #feat_weights = np.append(feat_weights, [1])
    
    # all visual features have the same weight
    vis_feat_weights = np.ones(vis_feat.size)*4.0*vis_feat.size/(1.0*feat_array.size)
    # all feature classes have the same weight
    #vis_feat_weights = np.ones(30*128)*vis_feat.size/(1.0*feat_array.size)
    #vis_feat_weights = np.append(vis_feat_weights, np.ones(30*128)*vis_feat.size/(1.0*feat_array.size))
    #... TODO
    
    feat_weights = np.append(vis_feat_weights, [1.0/(1.0*feat_array.size)])
    feat_weights = np.append(feat_weights, [1.0/(1.0*feat_array.size)]) 
    feat_weights = np.append(feat_weights, np.ones(prv_ctrl_feat.size)*prv_ctrl_feat.size/(1.0*feat_array.size))
    feat_weights = np.append(feat_weights, [1.0/(1.0*feat_array.size)])
    
    return (feat_array, feat_weights)

