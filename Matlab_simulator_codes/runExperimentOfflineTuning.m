
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%                                                        %
%  runExperimentOfflineTuning.m                          %
%  Created: Stephen Tully, CMU ECE/RI                    %
%  Contact: stully@ece.cmu.edu, stephen.tully@gmail.com  %
%  Date: 2013.06.12                                      %
%                                                        %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% initialization
clear; close all;
figure(1); cla; hold on;
axis equal;

% some constants
linkLengthsTurning = 1;
Tregister = eye(4);
numDatasets = 3;

% the important logged files
logFileName{1} = 'data/2013.05.03.benchHeartData/experimentLogTrial1.log';
trailFileName{1} = 'data/2013.05.03.benchHeartData/_trailPointsTrial1mat.dat';
trailPoints{1} = loadTrailPoints(trailFileName{1});
logFileName{2} = 'data/2013.05.03.benchHeartData/experimentLogTrial2.log';
trailFileName{2} = 'data/2013.05.03.benchHeartData/_trailPointsTrial2mat.dat';
trailPoints{2} = loadTrailPoints(trailFileName{2});
logFileName{3} = 'data/2013.06.04.headDummyExperiment/log1.log';
trailFileName{3} = 'data/2013.06.04.headDummyExperiment/_trailLog1mat.dat';
trailPoints{3} = loadTrailPoints(trailFileName{3});
% logFileName{4} = 'data/2013.04.30.benchHeartData/experimentLog.log';
% trailFileName{4} = 'data/2013.04.30.benchHeartData/trailLogMat.dat';
% trailPoints{4} = loadTrailPoints(trailFileName{3});
% logFileName{5} = 'data/2010.03.15.pigData/autonomousTest.log';
% trailFileName{5} = 'data/2010.03.15.pigData/trailLog1mat.dat';
% trailPoints{5} = loadTrailPoints(trailFileName{4});

% load the logged data
try
    disp('Loading the data from the saved MAT file...');
    load('savedData.mat');
    disp('   Done.');
catch
    disp('   Could not load file.');
    disp('Instead, loading the data from the log file...');
    for i = 1:numDatasets,
        [linkRadius{i}, linkLength{i}, numTimeSteps{i}, XKalm{i}, PKalm{i}, guidedPaused{i}, prevGuidedMode{i}, ...
            currGuidedMode{i}, phis{i}, thetas{i}, trackerPoses{i}] = loadLoggedData(logFileName{i});
    end
    disp('   Done.');
    disp('Saving data to a MAT file...');
    save('savedData.mat','linkRadius', 'linkLength', 'numTimeSteps', 'XKalm', ...
        'PKalm', 'guidedPaused', 'prevGuidedMode', 'currGuidedMode','phis','thetas','trackerPoses');
    disp('   Done.');
end

try
    disp('Loading the prior best parameters MAT file...');
    load('savedBestParams.mat');
    disp('   Done.');
catch
    
end

% some constants for initialization
params(1,1) = 0.1597;
params(2,1) = 0.0012;
params(3,1) = 0.0028;
params(4,1) = 0.0027;
params(5,1) = 0.0076;
params(6,1) = 6.9000;
bestParams = params;
bestAverageError = Inf;

% loop forever
done = 0;
firstLoop = 1;
while ~done,

    if (firstLoop)
        firstLoop = 0;
    else
        % choose a random parameter to change
        params = bestParams;
        q = randi(length(params),1);
        if (q == 1)
            params(q) = bestParams(q) + 1e-3.*randn(1);
        elseif (q == 6)
            params(q) = bestParams(q) + 1e-1.*randn(1);
        else
            params(q) = bestParams(q) + 1e-5.*randn(1);
        end
    end    
    measurementSpatialUncertainty = params(1);
    measurementAngularUncertainty = params(2);
    steerAngularUncertaintyPhi = params(3);
    steerAngularUncertaintyTheta = params(4);
    angleInitUncertainty = params(5);
    linkLength = params(6);
        
    % run through each time step for each experiment and run the filter
    sumErrorOverDatasets = 0;
    for s = 1:numDatasets,
        measurementPoints = [];
        Xoff = XKalm{s}{1};
        Poff = PKalm{s}{1};
        Poff(6:16,6:16) = angleInitUncertainty.*eye(11);
        for i = 2:numTimeSteps{s},
         % advance, if necessary
            if (length(XKalm{s}{i}) > length(XKalm{s}{i-1}))
                Xoff = [Xoff; 0.0; 0.0];
                Poff = [Poff, zeros(length(Poff(:,1)),2); zeros(2,length(Poff(:,1))), [steerAngularUncertaintyTheta 0; 0 steerAngularUncertaintyTheta]];
            end
            
            % apply steering motion model
            if (length(Xoff) > 4)
                phis{s}(i) = phis{s}(i)/2.0;
                [Xoff, Poff] = predictSteering(Xoff, Poff, phis{s}(i), thetas{s}(i), linkLengthsTurning, steerAngularUncertaintyPhi, steerAngularUncertaintyTheta);
                measurementPoints = [measurementPoints, trackerPoses{s}{i}(1:3,4)];
                [Xoff, Poff] = correctSteering(Xoff, Poff, trackerPoses{s}{i}, measurementAngularUncertainty, measurementSpatialUncertainty, linkLength);
            else
                Xoff = XKalm{s}{i};
                Poff = PKalm{s}{i};
            end
            
        end
        
        % plot the trail points and estimated snake points
        cla;
        plot3(trailPoints{s}(:,1),trailPoints{s}(:,2),trailPoints{s}(:,3),'k-');
        [snakePoints] = computeSnakePoints(Xoff,linkLength,Tregister);
        plot3(snakePoints(:,1),snakePoints(:,2),snakePoints(:,3),'r-');
        plot3(measurementPoints(1,:),measurementPoints(2,:),measurementPoints(3,:),'m.');
        
        % find average/max error
        worstError = 0.0;
        sumErrors = 0.0;
        for i = 1:length(snakePoints(:,1)),
            bestError = Inf;
            for j = 1:length(trailPoints{s}(:,1)),
                dist = norm(trailPoints{s}(j,:)-snakePoints(i,:));
                if (dist < bestError)
                    bestError = dist;
                end
            end
            if (bestError > worstError)
                worstError = bestError;
            end
            sumErrors = sumErrors + bestError;
        end
        averageError(s) = sumErrors/length(snakePoints(:,1));
        sumErrorOverDatasets = sumErrorOverDatasets + averageError(s);
    end
    if (sumErrorOverDatasets < bestAverageError)
        for s = 1:numDatasets,
            disp(['Best Average error = ', num2str(averageError(s)), ' for experiment ', num2str(s)]);
        end
        bestAverageError = sumErrorOverDatasets;
        bestParams = params;
        save('savedBestParams.mat','bestAverageError','bestParams');
    end
    disp(' ');
    pause(0.000001);
end
