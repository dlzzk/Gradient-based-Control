function invAdg = Adg_inv(g)
% A helper function used in finding the body Jacobian, finds the Adj inv
% Input: g, a 4*4 transformation matrix
% Output: invAdj, inverse of the Adjoint transformation matrix

R = g(1:3,1:3);
p = g(1:3,4);
p_skew = skew(p);
invAdg = [R',-R'*p_skew;zeros(3,3),R'];

end



