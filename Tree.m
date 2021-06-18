classdef Tree
    %%%%USAGE: To instantiate a Tree object
    %%%%% tree = Tree(qstart);
   properties (SetAccess = private)
        Nodes = {[]}; %array of nodes
        Parents = [0]; % list of nodes' parents indices       
   end
  
   methods 
      %constructor; init with root node
      function [obj] = Tree(root)
          obj.Nodes{end} = root;
      end 
      
      function [obj, qnewID ]= addEdge(obj, parentID, child)
        % USAGE: tree = tree.addEdge(parentID, child) create a new edge
        % with child as new node. Return the modified tree and ID of child.


        if parentID < 0 || parentID > numel(obj.Parents)
            error('MATLAB:tree:addnode', ...
                'Cannot add to unknown parent with index %d.\n', parent)
        end

        obj.Nodes{end +1} = child;          
        obj.Parents = [obj.Parents, parentID ];
        qnewID = numel(obj.Nodes);
      end
      
      
      function nns = KNN(obj, node, k)
%         returns a list of IDs of k nodes in the tree nearest to node
%         convert node cell array to nx4 matrix
        mat = cell2mat(obj.Nodes);
        node_matrix = reshape(mat, 4, []).';
%        search for the indices of k nearest nodes 
        nns = knnsearch(node_matrix,node, 'K', k);     
      end
      
      function parentID = getParentID(obj, nodeID)
          %returns the parent ID of node
          parentID = obj.Parents(nodeID);
      end
      
     function path = retracePathFrom(obj, nodeID)
          %return path (sequence of nodes) from root to nodeID
          path = [obj.Nodes{nodeID}];
          parentID = obj.Parents(nodeID);
          while parentID ~= 0
              path = [path; obj.Nodes{parentID}];
              parentID = obj.Parents(parentID);
          end
          path = flip(path, 1);
                
      end
      
      function cost = getCostTo(obj, nodeID)
          %return the cost of path from start to node pointed to by nodeID  
          path = obj.retracePathFrom(nodeID);
          normOfDiffs = vecnorm(transpose(diff(path))); 
          cost = sum(normOfDiffs);
          shape =size(path);
          if shape(1) < 2
              cost = 0;
          end
      end
      
      function NNids = getNays(obj, q, r)
          %return a set of nodes in the hyperball of radius r, centered at
          %q
          nodes = reshape(cell2mat(obj.Nodes), numel(obj.Nodes{1}), numel(obj.Nodes));
          nodes = transpose(nodes);
          normOfDiffs = vecnorm(transpose(nodes - q));
          NNids = find(normOfDiffs <= r);
      end
      
      function parentID = chooseParent(obj, qnew, nayIDs, map)
          %returns parent's ID of qnew such that the cost
          %thru this parent is lowest         
          parentID = nayIDs(1);
          costToNay = obj.getCostTo(nayIDs(1));

          intCost = norm(qnew- obj.Nodes{nayIDs(1)});  
          minCost = costToNay + intCost;
          
          for ind = nayIDs
              costToNay = obj.getCostTo(ind);
              intCost = norm(qnew - obj.Nodes{ind});
              candidateCost = costToNay + intCost;
              if candidateCost < minCost && checkBranchValid(obj.Nodes{ind},qnew, map, 20, false)
                  minCost = candidateCost;
                  parentID = ind;
              end        
          end
      end
      
      function obj = rewire(obj, qnewID, nayIDs, map)
          %return a rewired tree where for each node in nodeIDs,
          %its parent changes to qnew if doing so results in a lower cost
          %from the root
          costToqNew = obj.getCostTo(qnewID);
          for i = 1:numel(nayIDs)
              oldCost = obj.getCostTo(nayIDs(i));
              candidateCost = costToqNew + norm(obj.Nodes{qnewID}-obj.Nodes{nayIDs(i)});
              if candidateCost < oldCost && checkBranchValid(obj.Nodes{nayIDs(i)}, obj.Nodes{qnewID}, map, 20, false)
                  obj.Parents(nayIDs(i)) = qnewID; %then change parent
                  disp("REWIRING....");
              end
          end       
      end
      
      
      function [mincost, minID ]= minCost(obj, nearGoals)
          %returns minimum cost of goal.
          %nearGoals is a list of close-to-goal nodeIDs
          mincost = obj.getCostTo(nearGoals(1));
          minID = nearGoals(1);
          for id = nearGoals(2: end)
              cost = obj.getCostTo(id);
              if cost < mincost
                  mincost = cost;
                  minID = id;
              end
          end
      end
      
   end
    
end