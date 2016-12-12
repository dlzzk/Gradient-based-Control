function w_skew = skew(w)
% w is a 3*1 vector
% w_skew is the skew symmetric matrix expansion of w
x = w(1);
y = w(2);
z = w(3);
w_skew = [0 -z y;
          z 0 -x;
          -y x 0];
     
end