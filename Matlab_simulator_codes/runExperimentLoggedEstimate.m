
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%                                                        %
%  runExperimentLoggedEstimate.m                         %
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

% open the data file
dataFile = fopen('data/2013.05.03.data/experimentLogTrial1.log','r');
if (dataFile < 0)
    disp('Error 0: could not open data file.');
    close all;
    return;
end

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
pause

done = 0;
while ~done,
    line = fgetl(dataFile);
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
        hFullKalm = drawState(state,[0 1 0],linkLength,linkRadius,1,eye(4),0);
        axis equal
        pause(0.001);
    end
end


% close the data file
fclose(dataFile);


