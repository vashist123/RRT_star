function [path, cost]= postProcess(path, map)
    %returns postProcessed path and its cost
    n = size(path);
    head = 1;
    final_ids = head;
    
    for i = 1:n(1)-1
        validBranch = checkBranchValid(path(head, :),path(i+1, :), map, 20*i, true);     
        if ~validBranch
            body = i;
            tail = i+1;
            final_ids = [final_ids, [body, tail]];
            head = tail;
        end              
    end

    %append goal config if its not included in final path
    if norm(path(final_ids(end), :) - path(end, :)) > 0.01
        final_ids(end+1) = n(1);
    end
    
    path = path(final_ids, :);
    
    %compute cost of path from only its first 4 columns
    p = path(:,1:4);
    normOfDiffs = vecnorm(transpose(diff(p))); 
    cost = sum(normOfDiffs);
    shape =size(path);
    if shape(1) < 2
    cost = 0;
    end
    

end