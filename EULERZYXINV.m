function rotate_angles = EULERZYXINV(r)
% input r is the rotation matrix
      beta = asin(r(1,3));
      gamma = atan2(-r(1,2),r(1,1));
      alpha = atan2(-r(2,3),r(3,3));
    
      rotate_angles = [alpha;beta;gamma];
end