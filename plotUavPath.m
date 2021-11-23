function plotUavPath(waypoints, A, base, N, W, S, E)
%plotUavPath Summary of this function goes here
%   Detailed explanation goes here

% A = imread('hangar.jpg');
test = waypoints;
mapWidth = abs(deg2km(E-W)*1000);
mapHeight = abs(deg2km(N-S)*1000);
Rx = base(1)/size(A,2);
Ry = base(2)/size(A,1);

if length(waypoints) > 6
    clr = prism(length(waypoints));
else
    clr = [1 0 0;
           0 1 0;
           0 0 1;
           1 1 0;
           1 0 1;
           0 1 1];
end

for k = 1:length(waypoints)
    i = 1;
    for i = 1:length(waypoints{k})
        %pause(0.25);
        plot(waypoints{k}(i,1),waypoints{k}(i,2),'o','Color',clr(k,:),...
             'LineWidth',4,'MarkerSize',8,'MarkerFaceColor',clr(k,:))
        if i > 1
            plot(waypoints{k}(i-1:i,1),waypoints{k}(i-1:i,2),'Color',clr(k,:),...
                 'LineWidth',4);
        end
        % Remap the path node to the GPS coordinates
        test{k}(i,1)= W + km2deg((waypoints{k}(i,1)+Rx*mapWidth)/1000);
        test{k}(i,2)= N - km2deg((waypoints{k}(i,2)+Ry*mapHeight)/1000);
    end
    
    % Save path into KML files
    filename = strcat('GEUAV',int2str(k),'.kml');
    UAVname = strcat('UAV',int2str(k));
    kmlwriteline(filename, test{k}(:,2), test{k}(:,1),...
        'Name', UAVname,'Color',clr(k,:), 'LineWidth', 5)
end

