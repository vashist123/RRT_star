function [path, costsToGoal] = rrt_star_pp(map, start, goal)
% RRT Find the shortest path from start to goal.
%   PATH = rrt(map, start, goal) returns an mx6 matrix, where each row
%   consists of the configuration of the Lynx at a point on the path. The
%   first row is start and the last row is goal. If no path is found, PATH
%   is a 0x6 matrix.
%
% INPUTS:
%   map     - the map object to plan in
%   start   - 1x6 vector of the starting configuration
%   goal:   - 1x6 vector of the goal configuration


%% Prep Code

path = [];

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%                  Algortihm Starts Here             %%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%%%%% RRT* with Post-Processing %%%%%%%%

originalStart = start;
originalGoal = goal;
start = start(1:4);
goal = goal(1:4);
tree = Tree(start); %initialize a tree
%hyper params to be adjusted
epsilon = 0.5; %qrand-goal tolerance
goalFound = false;
nearGoalIDs = []; %list of nodeIDs near goal
numIter = 1800; %number of iterations
costsToGoal = zeros(2,numIter);

%iterate until cost to goal converges
    %for iter = 1:numIter
    iter = 1;
    while ~goalFound
%         1. Sample from C-space
        if goalFound
            [minCostToGoal, minGoalID] = tree.minCost(nearGoalIDs); 
            path = tree.retracePathFrom(minGoalID);
            qrand = sampleNearPath(path);            
          
        else
            qrand = sample(goal,goalFound );

        end


        %2. Find closest node to qrand
        closestID = tree.KNN(qrand, 1);
        qnear = tree.Nodes{closestID};
        qnew = steer(qnear, qrand, numel(tree.Nodes));
        %4. Check if goal is near
        if norm(qnew-goal) < epsilon
            qnew = goal;
            branchIsValid = checkBranchValid(qnear,qnew, map, 20, false);
            if branchIsValid
                %%%%%%GOAL REACHED HERE%%%%%%%%%
                if ~goalFound
                    disp("GOAL REACHEDDDDDDDD");
                end
                goalFound = true;
                
                %5. CHOOSE BEST PARENT FOR qnew
                n = numel(tree.Nodes);
                radius = 5.0*(log(n)/n)^(1/4);
                nearIDs = tree.getNays(qnew, radius);
                nearIDs = [closestID, nearIDs]; %append closestID to nearID
                bestparentID = tree.chooseParent(qnew, nearIDs, map);
                %6. insert qnew and append qnewID(newgoalID)
                [tree, qnewID] = tree.addEdge(bestparentID, qnew);
                nearGoalIDs = [nearGoalIDs, qnewID]; %add qnewID to nearGoalIDs 
                %7. Rewire Tree around qnew
                tree = tree.rewire(qnewID, nearIDs, map);
                
            end
        else
            branchIsValid = checkBranchValid(qnear, qnew, map, 20, false);
                if branchIsValid
                    n = numel(tree.Nodes);
                    radius = 5.0*(log(n)/n)^(1/4);
                    nearIDs = tree.getNays(qnew, radius);
                    nearIDs = [closestID, nearIDs]; %append closestID to nearID
                    bestparentID = tree.chooseParent(qnew, nearIDs, map);
                    [tree, qnewID] = tree.addEdge(bestparentID, qnew); %add qnew to tree  
                    %6. perform Rewiring around qnew
                    tree = tree.rewire(qnewID, nearIDs, map);
                end      
        end
        
        
        if goalFound
            [minCostToGoal, minGoalID] = tree.minCost(nearGoalIDs); 
            fprintf("%d || Cost to Goal: %d \n", iter, minCostToGoal);
            %store costToGoal for plotting
            costsToGoal(1, iter) = iter;
            costsToGoal(2, iter) = minCostToGoal;
            
            path = tree.retracePathFrom(minGoalID);
            r = size(path);
            r = r(1);
            z = ones(2, r);
            z(1, :) = z(1, :)*originalStart(5);
            z(2, :) = z(2, :)*originalStart(6);
            path = transpose([transpose(path); z]);
            path(end, 5) = originalGoal(5);
            path(end, 6) = originalGoal(6);          
        end 
        iter = iter +1;
    end
    %adjust starting iter so that plot of cost begins after initial path is
    %found
    startInd = find(costsToGoal(2,:) == 0, 1, 'last' ) + 1;
    costsToGoal = costsToGoal(:, startInd:end);
    
    
    %post-process path and compute its cost
    [path, ppCost]  = postProcess(path, map);
    fprintf("Post Processed Cost to Goal: %d \n", ppCost);



%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%                  Algortihm Ends Here               %%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


end
