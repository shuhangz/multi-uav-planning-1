function [lmin,lmax,V,laneDist] = findStrips(x,y,sidelap,imageWidth,imageLength)

% Calculate the Convex Hull of the region to be covered.
k = convhull(x,y);

% Plot the region and the Convex Hull.
% plot(x(k),y(k),'k-')
plot([x; x(1)],[y; y(1)],'k+:')

% From this point on, the only points that matter are those
% that define the Convex Hull.
x = x(k);
y = y(k);

% Rotating the ROI to find the sweep angle
% that uses the least number of lines.
areaWidth = max(x)-min(x);
thetamin = 0;
for i = 1:1:360
    theta = i*2*pi/360;
    R = [cos(theta) -sin(theta); sin(theta) cos(theta)];
    aux = R*[x y]';
    if (max(aux(1,:))-min(aux(1,:))) < areaWidth
        areaWidth = max(aux(1,:))-min(aux(1,:));
        thetamin = theta;
    end
end 
thetamin
% Rotates the region to the angle chosen in the previous step
% to facilitate subsequent calculations.
R = [cos(thetamin) -sin(thetamin); sin(thetamin) cos(thetamin)];
aux = R*[x y]';
x = aux(1,:)';
y = aux(2,:)';

areaWidth = max(x)-min(x);
areaLength = max(y)-min(y);
numberOfLanes = ceil(areaWidth/(imageWidth*(1-sidelap)));
laneDist = areaWidth/(numberOfLanes);
for i = 1:numberOfLanes
    xi = min(x)+ ...
         laneDist*i-laneDist/2;
    delta = areaLength/imageLength;
    k = 0;
    miny = min(y) + k*delta;
    while ~inpolygon(xi,miny,x,y)
        miny = min(y) + k*delta;
        k = k + 1;
    end
    
    k = 0;
    maxy = max(y) - k*delta;
    while ~inpolygon(xi,maxy,x,y)
        maxy = max(y) - k*delta;
        k = k + 1;
    end
    
    lanemin(i,:) = [ xi miny ];
    lanemax(i,:) = [ xi maxy ];
end

lmin = (R'*lanemin')';
lmax = (R'*lanemax')';

% construction of the vertices
V = zeros(numberOfLanes*2+1,2);
for i = 1:numberOfLanes*2+1
    if i == 1
        V(i,:) = [0 0];
    elseif mod(i,2) == 0
        V(i,:) = lmin(i/2,:);
    else
        V(i,:) = lmax((i-1)/2,:);
    end
end
