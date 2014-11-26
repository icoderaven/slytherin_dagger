function [linkRadius, linkLength, numTimeSteps, XKalm, PKalm, guidedPaused, prevGuidedMode, currGuidedMode, phis, thetas, trackerPoses] = loadLoggedData(filename)

% load the link radius and link length parameters
dataFile = fopen(filename,'r');
if (dataFile == -1), error('Error: Could not open log file.'); end;    
line = fgetl(dataFile);
[~, line] = strtok(line);
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
end

% load and display the logged data
done = 0;
timeStep = 0;
while ~done,
    line = fgetl(dataFile);
    if (line == -1)
        done = 1;
        break;
    end
    for i = 1:3,
        [token, line] = strtok(line);
    end
    if (strcmp(token,'guidedMode') == 1),
        timeStep = timeStep + 1;
        [~, line] = strtok(line);
        [token, line] = strtok(line);
        if (strcmp(token,'guidedPaused') ~= 1), error('Error 3: parsing the file failed.'); end;
        [token, line] = strtok(line);
        guidedPaused(timeStep,1) = str2num(token);
        [token, line] = strtok(line);
        if (strcmp(token,'prevGuidedMode') ~= 1), error('Error 4: parsing the file failed.'); end;
        [token, line] = strtok(line);
        prevGuidedMode(timeStep,1) = str2num(token);
        [token, line] = strtok(line);
        if (strcmp(token,'currGuidedMode') ~= 1), error('Error 5: parsing the file failed.'); end;
        [token, line] = strtok(line);
        currGuidedMode(timeStep,1) = str2num(token);
    elseif (strcmp(token,'snakeEstimation') == 1),
        [token, line] = strtok(line);
        if (strcmp(token,'numFullLinks') == 1)
            [token, line] = strtok(line);
            numFullLinks = str2num(token); 
        else
            error('Error 6: parsing the file failed.');
        end
        state = zeros(2*numFullLinks+4,1);
        cov = zeros(2*numFullLinks+4,2*numFullLinks+4);
        if (numFullLinks > 0)
            [token, line] = strtok(line);
            if (strcmp(token,'full_kalm_updating') ~= 1), error('Error 7: parsing the file failed.'); end;
            [token, line] = strtok(line);
            if (strcmp(token,'1') ~= 1), error('Error 8: parsing the file failed.'); end;
            [token, line] = strtok(line);
            if (strcmp(token,'X_full_kalm') ~= 1), error('Error 9: parsing the file failed.'); end;
            for i = 1:(2*numFullLinks+4),
                [token, line] = strtok(line);
                state(i) = str2num(token);
            end
            [token, line] = strtok(line);
            if (strcmp(token,'P_full_kalm') ~= 1), error('Error 10: parsing the file failed.'); end;
            for i = 1:(2*numFullLinks+4)^2,
                [token, line] = strtok(line);
                cov(i) = str2num(token);
            end
        end
        XKalm{timeStep} = state;
        PKalm{timeStep} = cov';
    elseif (strcmp(token,'phi') == 1),
        [token, line] = strtok(line);
        phis(timeStep,1) = str2num(token);
        [token, line] = strtok(line);
        if (strcmp(token,'theta') ~= 1), error('Error 11: parsing the file failed.'); end;
        [token, line] = strtok(line);
        thetas(timeStep,1) = str2num(token);
    elseif (strcmp(token,'ascensionTracker') == 1),
        [~, line] = strtok(line);
        [~, line] = strtok(line);
        [~, line] = strtok(line);
        [~, line] = strtok(line);
        [token, line] = strtok(line);
        if (strcmp(token,'Tsensor') ~= 1), error('Error 12: parsing the file failed.'); end;
        Tascension = zeros(4,4);
        for i = 1:16,
            [token, line] = strtok(line);
            Tascension(i) = str2num(token);
        end
        trackerPoses{timeStep} = Tascension';
    end
    
end
numTimeSteps = timeStep;
fclose(dataFile);

