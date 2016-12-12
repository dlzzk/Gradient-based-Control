function e_ksi_skew_theta_generay = get_ur5_forward_kinematics(theta,size)
% INPUT: theta is an array which contains size values of theta
% OUTPUT: e_ksi_skew_theta is the forward kinematics of ur5 which has the
% equation: gst = e_ksi_skew_theta * gst0

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
    e_ksi_skew_theta{size} = zeros(4,4);
    for i = 1:size
        e_ksi_skew_theta{i} = [e_w_skew_theta{i} (eye(3)-e_w_skew_theta{i})*cross(w(:,i),v(:,i))+w(:,i)*w(:,i)'*v(:,i)*theta(i);
                               0 0 0 1];
    end
    
    e_ksi_skew_theta_generay = eye(4,4);
    for i = 1:size
        e_ksi_skew_theta_generay = e_ksi_skew_theta_generay * e_ksi_skew_theta{i};
    end
end