function branchIsValid = checkBranchValid(qnear,qnew, map, n, isPP)
    %Inputs: 1. qnear, a node already in tree; 
    %2. qnew, a candidate node to branch to;
    %Returns a boolean whether this branch is valid
    %n: the number of steps
    
    sweep = qnear.' + linspace(0,1,n).*(qnew.' - qnear.');
    %check validity of all intermediate configs between qnear and qnew
    Q = zeros(1,6);
    for q=  sweep
        q = transpose(q);
        %in postProcessing brach check, q id 6-d
        if isPP
            Q(1:6) = q;
        else
            Q(1:4) = q; %append zero orientation
        end
        
        if isRobotCollided(Q, map)
            branchIsValid = false;
            return
        end  
    end   
    branchIsValid = true;
    
end