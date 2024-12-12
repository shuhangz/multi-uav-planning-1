close all
clear
% clc
addpath(genpath("C:\soft\YALMIP-master"));


%% Configurations
% UAV
global O
global uavNumber;
global uavSetupTime;
global uavFlightTime;
O = 1; % O - Operators
uavNumber = 2; % M - Total Number
uavSetupTime = 2;  % ts - average of 8.2 minutes of setup
uavFlightTime = 5; % Lk - Battery Duration

uavSpeed = 45; % Vij - Flight Speed Const
%flightAltitude = 240;
%flightAltitude = 210;
% flightAltitude = 180; % H - Flighting Height
flightAltitude = 50;

% Heuristic 0: No but iteration
%           1: Yes and no iteration
h = 0;

% Camera specs
horizontalResolution = 4608;
verticalResolution = 3456;

% Typical 35 mm FOV
% 18mm h: 100 v: 66;
% 28mm h: 74 v: 49;
% 35mm h: 62 v: 41;
hfieldOfView = 74;
vfieldOfView = 49;
sidelap = .5; % s - image overlap

% base = [897.5000  491.5000];
% N = -20.72613271;
% W = -43.81682358;
% S = -20.74471033;
% E = -43.77901375;

A = imread('cuhk.png');
base = [1595 1368];
N = 22.4240;
W = 114.2003;
S = 22.4177;
E = 114.2096;

%% Parameters
imageWidth = flightAltitude*2*tan(pi*(hfieldOfView/2)/180);
imageLength = flightAltitude*2*tan(pi*(vfieldOfView/2)/180);

%% Program
plotMap(A, base, N, W, S, E);
% Select the boundary points of the area to be investigated. 
% The base is the first chosen point and the other points 
% (minimum of three in total) will be used to define the boundary.
[x,y] = ginput;
% Examples:
%x = [500 500 1000 1000]'; y = [-250 250 -250 250]';
%x = [490 490 1510 1510]'; y = [500 -500 -500 500]';    % article
%x = [490 490 1510]'; y = [500 -500 500]';              % article
%x = [490 400 490 1000 1510 1400 1510 1000]'; y = [500 0 -500 -535 -500 0 500 575]'; %article
% x = [-2000 -2000 2000]'; y = [-500 -1000 -500]';
%x = 1.0e+03 * [0.370 0.251 2]';y = [-278 217 -200]';
%x = 1.0e+03 * [0.370 0.132 2]';y = [-278 722 -200]';
%x = [170 -101 561 337]';y = [ -297 727 -249 689]';
%x = [-325.3297 909.2549 732.8857 -525.5327]';y = [-406.7732 -201.8035 966.0468 718.1765]';
%x = [-225 165 1100 850]';y = [784 -716 -502 808]';
%x = [250 250 975]';y = [-300 725 725]';

%%
[lmin,lmax,V,laneDist] = findStrips(x,y,sidelap,imageWidth,imageLength);
plotStrips(lmin,lmax);
drawnow;
%%
uav = ones(uavNumber,1);
if uavNumber == 1
    uav(2,1) = 0;
end

[X,v,m] = lip3(V,uav,uavSpeed,uavSetupTime,uavFlightTime,h,O);
if m > 1 && h == 0
    while m > 1
        [ans,uavMaxTimeIndex] = max(v);
        uav(uavMaxTimeIndex) = 0; % remove the UAV with longest path
        for i = 1: length(v)
            if v(i) == 0
                uav(i) = 0;
            end
        end
        
        waypoints{uavMaxTimeIndex} = findWaypoints(X(:,:,uavMaxTimeIndex),V);
        beep
        % Focus on Node, so select any Node has been visited by ijmax
        [visitedWaypoints,order] = max(X(:,:,uavMaxTimeIndex),[],1);
        j = 1; newV = [];
        for i = 1:length(V)
            if i == 1 % Base/Depot
                newV(j,:) = [0 0];
                j = j + 1;
            elseif visitedWaypoints(i) == 0 % Not visited by uavMax
                newV(j,:) = V(i,:);
                j = j + 1;
            end
        end
        V = newV;
        
        [X,v,m] = lip3(V,uav,uavSpeed,uavSetupTime,uavFlightTime,h,O);
    end
    [ans,uavMaxTimeIndex] = max(v);
    uav(uavMaxTimeIndex) = 0;
    for i = 1: length(v)
        if v(i) == 0
            uav(i) = 0;
        end
    end
    
    waypoints{uavMaxTimeIndex} = findWaypoints(X(:,:,uavMaxTimeIndex),V);
else
    for i = 1:m
        waypoints{i} = findWaypoints(X(:,:,i),V);
    end
end

laneDist
[t, t_fly] = time(waypoints,uavSpeed,uavSetupTime,O)

plotUavPath(waypoints, A, base, N, W, S, E);
%distance
beep
%plotForPaper;