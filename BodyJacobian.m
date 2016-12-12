function [Jb] = BodyJacobian(theta)
% The function calculates the body Jacobian of UR5
% Input: theta is the angular position of the six joints, 1*6 or 6*1
% Output: Jb is the body Jacobian matrix
%% Forward Kinematics Parts, Preparation for Jacobian Calculations
% Product of exponential of 6 joints
size = 6;
% Find the rotation axis w
w1 = [0;0;1]; 
w2 = [-1;0;0];
w3 = [-1;0;0];
w4 = [-1;0;0];
w5 = [0;0;1];
w6 = [-1;0;0];
w = [w1 w2 w3 w4 w5 w6];

% Find values for q
l0 = 89.2/1000;
l1 = 425/1000;
l2 = 392/1000;
l3 = 109.3/1000;
l4 = 94.75/1000;

q1 = [0;0;0];
q2 = [0;0;l0];
q3 = [0;0;l0+l1];
q4 = [0;0;l0+l1+l2];
q5 = [-l3;0;0];
q6 = [0;0;l0+l1+l2+l4];
q = [q1 q2 q3 q4 q5 q6];

% Calculate for v and ksi
v = zeros(3,6);
ksi = zeros(6,6);
for i = 1:6
    v(:,i) = cross(-w(:,i),q(:,i));
    ksi(:,i) = [v(:,i);w(:,i)];
end

% Calculate exp(w_skew*theta)
% First Calculate w_skew
w_skew{size}=zeros(3,3);
e_w_skew_theta{size}=zeros(3,3);
for i = 1:6
    w_skew{i} = skew(w(:,i));
    e_w_skew_theta{i} = eye(3)+w_skew{i}*sin(theta(i))+w_skew{i}*w_skew{i}*(1-cos(theta(i)));
end

% Calculate exp(ksi_skew*theta)
g{size} = zeros(4,4);
for i = 1:size
    g{i} = [e_w_skew_theta{i} (eye(3)-e_w_skew_theta{i})*cross(w(:,i),v(:,i))+w(:,i)*w(:,i)'*v(:,i)*theta(i);
            0 0 0 1];
end

% Calculate gst0
l5 = 82.5/1000;
gst0 = [eye(3) [-(l3+l5);0;l0+l1+l2+l4];0 0 0 1];

%% Calculate the body Jacobian - Adapted from book page 117
Jb = zeros(6,6);
for i = 1:6
    index = 7-i;
    cur_g = eye(4);
    for j = index:6
        cur_g = cur_g*g{j};
    end
    cur_g = cur_g * gst0;
    cur_ksi = Adg_inv(cur_g) * ksi(:,index);
    Jb(:,index) = cur_ksi; 
end

end