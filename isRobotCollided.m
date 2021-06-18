function [isCollided] = isRobotCollided(q, map, robot)
% ISROBOTCOLLIDED Detect if a configuration of the Lynx is in collision
%   with any obstacles on the map.
%
% INPUTS:
%   q   - a 1x6 configuration of the robot
%   map - a map strucutre containing axis-aligned-boundary-box obstacles
%   robot - a structure containing the robot dimensions and joint limits
%
% OUTPUTS:
%   isCollided - a boolean flag: 1 if the robot is in collision, 0 if not.

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%                  Algortihm Starts Here             %%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

    %1.Get robot's geometry in workspace
    [jpos, transform] = calculateFK_sol(q);
    segPos = [jpos; transpose(transform(1:3,4))]; %append effector's position to list of joint pos
    %2.For each link of the robot, bound it with a box. 
    %Define width and height of bounding box
    bwidth = 20;
    bheight = 20; % about 0.8 inch
    %check link 1
    B1 = [];
    B2 = [];
    p1 = segPos(1, :);
    p2 = segPos(2, :);
    w = [bwidth bwidth 0 ];
    h = [0 0 bheight];
%     p1_corners =  [p1+w+h; p1+w-h; p1-w+h; p1-w-h];
%     p2_corners =  [p2+w+h; p2+w-h; p2-w+h; p2-w-h];
%     B1 = [B1 ; p1; p1_corners];% stores one end of line seg
%     B2 = [B2; p2; p2_corners];%stores the other end of line seg. 
    %B1 and B2 are row-corresponding to comply with detectCollision()
    %usage.
    
    
    %build bounding boxes
    for i = 1:length(segPos)-1 %check link 1-6 
        p1 = segPos(i, :);
        p2 = segPos(i+1, :);
        %create 2 rectangles(4 corners)centered at p1 and p2
        %first define width vector w and height vector h
        %The 4 corners are then [p+w+h, p+w-h, p-w+h, p-w-h] 
%         w = cross(p1,p2);
%         w = bwidth*w/norm(w);
%         h = cross(p2, w);
%         h = bheight*h/norm(h);
        p1_corners =  [p1+w+h; p1+w-h; p1-w+h; p1-w-h];
        p2_corners =  [p2+w+h; p2+w-h; p2-w+h; p2-w-h];
        B1 = [B1 ; p1;p1_corners]; %append joint pos p1 and its surrounding 4 corners
        B2 = [B2; p2;p2_corners];
    end
%%%%CHECK whether a box edge intersects with any obstacles

    for obstacle = map.obstacles.'
        collided = detectCollision(B1, B2, obstacle);
        if any(collided)
            isCollided = true;
            return;
        end      
    end
    isCollided = false;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%                  Algortihm Ends Here               %%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

end
