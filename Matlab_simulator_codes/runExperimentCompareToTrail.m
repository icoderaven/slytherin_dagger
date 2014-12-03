
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%                                                        %
%  runExperimentCompareToTrail.m                         %
%  Created: Stephen Tully, CMU ECE/RI                    %
%  Contact: stully@ece.cmu.edu, stephen.tully@gmail.com  %
%  Date: 2012.02.01                                      %
%                                                        %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% initialization
clear; close all;
figure(1); cla; hold on;

% declare some constants
Tregister = eye(4);

% open the logged data file
dataFile = fopen('data/2013.05.03.data/experimentLogTrial1.log','r');
if (dataFile < 0)
    disp('Error 0: could not open data file.');
    close all;
    return;
end

% open the trail data
trailsData = load('data/2013.05.03.data/_trailPointsTrial1mat.dat');
trailPoints = trailsData(:,[5,9,13]);

% load the link radius and link length parameters
line = fgetl(dataFile);
[token, line] = strtok(line);
[token, line] = strtok(line);
if (strcmp(token,'SNAKE_LINK_LENGTH') == 1)
    [token, line] = strtok(line);
    linkLength = str2num(token);
else
    error('Error 1: parsing the file failed.');
end
[token, line] = strtok(line);
if (strcmp(token,'SNAKELIB_LINK_RADIUS') == 1)
    [token, line] = strtok(line);
    linkRadius = str2num(token);
else
    error('Error 2: parsing the file failed.');
end

for i = 1:5,
    [token, line] = strtok(line);
    disp(token)
end

% load and display the logged data
done = 0;
while ~done,
    line = fgetl(dataFile)
    if (line == -1)
        done = 1;
        break;
    end
    for i = 1:3,
        [token, line] = strtok(line);
    end
    if (strcmp(token,'snakeEstimation') == 1),
        [token, line] = strtok(line);
        if (strcmp(token,'numFullLinks') == 1)
            [token, line] = strtok(line);
            numFullLinks = str2num(token); 
        else
            error('Error 3: parsing the file failed.');
        end
        if (numFullLinks > 0)
            [token, line] = strtok(line);
            if (strcmp(token,'full_kalm_updating') ~= 1), error('Error 4: parsing the file failed.'); end;
            [token, line] = strtok(line);
            if (strcmp(token,'1') ~= 1), error('Error 5: parsing the file failed.'); end;
            [token, line] = strtok(line);
            if (strcmp(token,'X_full_kalm') ~= 1), error('Error 6: parsing the file failed.'); end;
            state = [];
            for i = 1:(2*numFullLinks+4),
                [token, line] = strtok(line);
                state = [state, str2num(token)];
            end
            try
                delete(hFullKalm)
            end
            [hFullKalm, snakePoints] = drawState(state,[0 1 0],linkLength,linkRadius,1,eye(4),0);
        end
        axis equal
        pause(0.001);
    end
end

% plot the trail points
plot3(trailPoints(:,1),trailPoints(:,2),trailPoints(:,3),'k-');
plot3(snakePoints(:,1),snakePoints(:,2),snakePoints(:,3),'r-');

% find average/max error
worstError = 0.0;
sumErrors = 0.0;
for i = 1:length(snakePoints(:,1)),
    bestError = Inf;
    for j = 1:length(trailPoints(:,1)),
        dist = norm(trailPoints(j,:)-snakePoints(i,:));
        if (dist < bestError)
            bestError = dist;
        end
    end
    if (bestError > worstError)
        worstError = bestError;
    end
    sumErrors = sumErrors + bestError;
end
disp(['Worst error = ', num2str(worstError)]);
disp(['Average error = ', num2str(sumErrors./length(snakePoints(:,1)))]);

% close the data file
fclose(dataFile);


