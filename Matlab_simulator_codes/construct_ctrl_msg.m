function msg = construct_ctrl_msg(msg,yaw,pitch)
tmp = msg.getLinear();
tmp.setX(yaw);
tmp.setY(pitch);
msg.setLinear(tmp);
