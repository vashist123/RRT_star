function norms = customNorm(diffMat)
      %returns norm of row vectors in diffMat.
      %The norm is computed by down weighted the 4,5, and 6th variables
      % The same weighting is used in distanceMetric.m
      mat4_5 = 0.75*diffMat(:,4:5);
      mat6 = (3/30)*diffMat(:, end);
      mat = [transpose(diffMat(:, 1:3)); transpose(mat4_5); transpose(mat6) ];
      norms = vecnorm(mat);
%       disp(size(norms));
end