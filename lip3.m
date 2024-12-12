function [X,v,m] = lip3(V,uav,uavSpeed,uavSetupTime,uavFlightTime,h,O)
% X - Xij indicator
% v - minimized cost (Optimization Objective)
% m - UAV number be used

% V - Vertices
% uav - uav Number
% h - Heuristic
% O - Operators

% construction of the cost matrix
C = zeros(length(V)); % C - Cost Matrix
numberOfVertices = length(V);
uavNumber = length(uav);

% Here is different from paper, Cij here is the Cij/Vij in Eq
for i = 1:numberOfVertices
    for j = 1:numberOfVertices
        if i ~= j
            C(i,j) = norm(V(i,:)-V(j,:))/(uavSpeed*1000/60); % Dist_ij/(m/s)
        end
    end
end

%% Linear Integer Programming
v = sdpvar(1,1); % symbolic decision variable 1x1 Mat
X = binvar(numberOfVertices,numberOfVertices,uavNumber,'full'); % i*j*k {0,1}
u = sdpvar(numberOfVertices,1);
m = intvar(1,1);
for k = 1:uavNumber
    % Xiik is always 0
    for i = 1:numberOfVertices
        X(i,i,k) = 0;
    end
    % No more UAV
    if uav(k) == 0
        X(:,:,k) = zeros(numberOfVertices);
    end
end


%% Restrictions

constraints = [];

% Constraint #1 - Total Cost of Operation
for k = 1:uavNumber
    sum1 = 0;
    for i = 1:numberOfVertices
        for j = 1:numberOfVertices
            if i ~= j
                sum1 = sum1 + C(i,j)*sum(X(i,j,k));
            end
        end
    end
    % Eq(6) sum(C/V*X)+dk<=v
    constraints = [constraints, v >= sum1 + sum(X(1,:,k))*ceil(k/O)*uavSetupTime];
    % Restriction 9 - Flight Autonomy
    % Eq(8) sum(C/V*X)<=Lk
    constraints = [constraints, sum1 <= uavFlightTime];
end

% Restrict 2 - Each vertex must be visited once and by a vant
% Eq(9) sum(xijk)=1
for j = 2:numberOfVertices
    constraints = [constraints, sum(sum(X(:,j,:))) == 1];
end

% Restriction 3 - When visiting a vertex, the vant must leave that vertex
% Eq(10) sum(xipk)-sum(xpjk)=0
for k = 1:uavNumber
    for p = 1:numberOfVertices
        constraints = [constraints, sum(X(:,p,k)) - sum(X(p,:,k)) == 0];
    end
end

% Restriction 4 and 5 - Number of vans used
% Eq(15)(16) sum(1jk)=m<=M
constraints = [constraints, sum(sum(X(1,:,:))) == m];
% if sum(uav) < length(uav)
%     constraints = [constraints, m == sum(uav)];
% else
%     constraints = [constraints, m <= uavNumber];
% end
constraints = [constraints, m <= sum(uav)];

% Restriction 6 - ???
% for k = 1:uavNumber
%     constraints = [constraints, sum(X(1,2:numberOfVertices,k)) == 1];
% end

% Restriction 7 - Cycle
for i = 2:numberOfVertices
    for j = 2:numberOfVertices
        if i ~= j
            constraints = [constraints, u(i) - u(j) + numberOfVertices*sum(X(i,j,:)) <= numberOfVertices - 1];
        end
    end
end

% Restriction 8 - The one that requires each line to be covered
% by only one UAV and in only one direction
% Eq(12) sum(Xii+1)+sum(Xi+1i)=1
for i = 2:2:numberOfVertices
   constraints = [constraints, 1 == sum(X(i,i+1,:)) + sum(X(i+1,i,:))];
end

% Restriction 10 - Avoid diagonals
% Eq(13)(14) 
for i = 2:2:numberOfVertices
   constraints = [constraints, sum(X(i,i+1,:)) == sum(sum(X(i,3:2:numberOfVertices,:)))];
   constraints = [constraints, sum(X(i+1,i,:)) == sum(sum(X(i+1,2:2:numberOfVertices,:)))];
end

% minimizes v subject to the constraints contained in the constraints variable
% with gurobi
% options = sdpsettings('solver','bnb','bnb.solver','fmincon');
% without gurobi
options = sdpsettings('verbose',1,'gurobi.Threads',4);
% cost func = v + /rou*mean(Tk)
if h == 1
    optimize(constraints,0.999*max(v)+0.001*mean(v),options);
    %optimize(constraints,max(v),options);
elseif h == 0
    optimize(constraints,v,options);
end

X = value(X);
for k = 1:uavNumber
    sum1 = 0;
    for i = 1:numberOfVertices
        for j = 1:numberOfVertices
            if i ~= j
                sum1 = sum1 + C(i,j)*sum(X(i,j,k));
            end
        end
    end
    v(k) = sum1 + sum(X(1,:,k))*k*uavSetupTime;
end
m = round(double(m));