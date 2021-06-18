function dist = distanceMetric(q1, q2)
    %special distance function that computes the distance between 
    %q1 and q2 reweighting the 4th, 5th, and last term(gripper width) so that the 
    %distance between orientations of the hand is downweighted by 0.6 and
    %the distance in the gripper component is in [0,3] range
    A = (q1(1:end-3)-q2(:, 1:end-3));
    A = [transpose(A); 
        transpose(0.75*(q1(end-3:end-1)-q2(:, end-3:end-1)));
        transpose((3/30)*(q1(end)-q2(:, end)))];
    A = transpose(A);
    dist = sqrt(sum(A.^2,2));
end