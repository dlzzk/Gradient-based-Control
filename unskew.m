function ksi = unskew(ksi_hat)
% a_skew is the 4*4 skew symmetric matrix expansion of ksi
% ksi is a 6*1 vector
ksi = zeros(6,1);
ksi(1:3,1) = ksi_hat(1:3,4);
ksi(4,1) = ksi_hat(3,2);
ksi(5,1) = ksi_hat(1,3);
ksi(6,1) = ksi_hat(2,1);
     
end