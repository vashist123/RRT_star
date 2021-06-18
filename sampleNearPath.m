function qrand = sampleNearPath(path)
    num_waypts = size(path);
    num_waypts = num_waypts(1);
    randIndx = randi([1, num_waypts-1]);
    
    alpha = rand;
    waypt1 = path(randIndx, :);
    waypt2 = path(randIndx+1, :);
    waypt = alpha*(waypt1) +(1-alpha)*(waypt2);
    sigma = 0.6*eye(numel(waypt));
    qrand = mvnrnd(waypt,sigma,1);

    %trim q_rand to be within joint limits;
    lowerLim = [-1.4000 -1.2000 -1.8000, -1.9000]; % -2 0];
    upperLim = [1.4000 1.4000 1.7000 1.7000 ]; % 1.5000 30];    
    lower_rand = max(qrand, lowerLim);
    qrand = min(lower_rand, upperLim);
    return;


end
