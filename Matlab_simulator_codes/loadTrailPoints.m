
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%                                                        %
%  loadTrailPoints.m                                     %
%  Created: Stephen Tully, CMU ECE/RI                    %
%  Contact: stully@ece.cmu.edu, stephen.tully@gmail.com  %
%  Date: 2013.06.12                                      %
%                                                        %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function [trailPoints] = loadTrailPoints(trailFileName)
% function [trailPoints] = loadTrailPoints(trailFileName)

trailData = load(trailFileName);
trailPoints = trailData(:,[5,9,13]);