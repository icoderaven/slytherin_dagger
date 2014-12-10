function [infered_action] = linear_predictor(feat_array,predictor)
% learned weights
weight = predictor(1,:);
% mean of features
mean_feat = predictor(2,:); 
% std of features
std_feat = predictor(3,:);
% mean of output (y-intercept)
mean_act = predictor(4,1);

% normalize the features
f = (feat_array - mean_feat)./std_feat;

% predict
infered_action = weight(:)'*f + mean_act;

end