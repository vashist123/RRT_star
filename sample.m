function q_rand = sample(goal, goalFound)
    if ~goalFound
        sigma = 0.8*eye(numel(goal));
        q_rand = mvnrnd(goal,sigma,1);
        %trim q_rand to be within joint limits;
        lowerLim = [-1.4000 -1.2000 -1.8000 -1.9000];%  -2 0];
        upperLim = [1.4000 1.4000 1.7000 1.7000 ];
        lower_rand = max(q_rand, lowerLim);
        q_rand = min(lower_rand, upperLim);
        return;
    else  
        q_rand = [(-1.4+2.8*rand), (-1.2+2.6*rand), (-1.8+3.5*rand), (-1.9+rand*3.6)];

    end
    
end