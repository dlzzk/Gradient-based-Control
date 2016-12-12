%% The script implements the Resolved Rate Control in UR5
%% Zhuokai Zhao, 12/12/2016
%% The first block of code is from ur5_example.m from Lab 0:
clear
tic
disp('Program started');
vrep=remApi('remoteApi');
vrep.simxFinish(-1);
id = vrep.simxStart('127.0.0.1', 19997, true, true, 2000, 5);

if id < 0
    disp('Failed connecting to remote API server. Exiting.');
    vrep.delete();
    return;
end

fprintf('Connection %d to remote API server open.\n', id);

% If your main code is run as a function, and not a script,
% you can use this command to ensure that cleanup_vrep is
% automatically run when there is a failure:
% cleanupObj = onCleanup(@() cleanup_vrep(vrep, id));

res = vrep.simxStartSimulation(id, vrep.simx_opmode_oneshot_wait);

% The following code is modified from ur5_init.m from Lab 0:
handles = struct('id', id);

[res, handles.ReferenceFrame] = vrep.simxGetObjectHandle(id, ...
    'ReferenceFrame', vrep.simx_opmode_oneshot_wait);
vrchk(vrep, res);

vrep.simxGetPingTime(id);

%% ************************************************************************
h = ur5_init(vrep, id);
startingJoints = zeros(1,6);

% Set the arm to its starting configuration (all zero):
for i = 1:6
    res = vrep.simxSetJointTargetPosition(id, h.ur5Joints(i),...
        startingJoints(i),...
        vrep.simx_opmode_oneshot);
    vrchk(vrep, res, true);
end

res = vrep.simxPauseCommunication(id, false);
vrchk(vrep, res);

% Make sure everything is settled before we start
pause(2);

% target location
l0 = 89.2/1000;
l1 = 425/1000;
l2 = 392/1000;
l3 = 109.3/1000;
l4 = 94.75/1000;
l5 = 82.5/1000;
gst0 = [eye(3) [-(l3+l5);0;l0+l1+l2+l4];0 0 0 1];
theta = [-pi/3;pi/6;-pi/4;pi/2;pi/2;-pi/4];
gsb = get_ur5_forward_kinematics(theta,6)*gst0;
% show the target location as a frame 
handles.NewFrame = copyf(id, vrep, gsb, handles.ReferenceFrame);

% Read current joint angular position
cur_pos = zeros(1,6);
for j = 1:6
    [res, cur_pos(j)] = vrep.simxGetJointPosition(id, h.ur5Joints(j),...
        vrep.simx_opmode_buffer);
    vrchk(vrep, res);
end
% get the current new gst
gst = get_ur5_forward_kinematics(cur_pos,6)*gst0;
% get gbt = inv(gsb) * gst
gbt = gst/gsb;
% take the logarithm of gbt to get ksi_hat
ksi_hat = real(logm(gbt));
% transform ksi_hat to ksi
ksi = unskew(ksi_hat);

while norm(gbt-eye(4)) > 0.001
    % Calculate the BodyJacobian
    Jb = BodyJacobian(cur_pos);
    % calculate q_dot = -inv(BodyJacobian)*ksi
    next_pos = -0.01*Jb' * ksi + cur_pos;
%     % q_dot times a little time interval delta_t to get new joint position q
%     cur_pos = cur_pos + q_dot * 0.01;
    % pass the updated cur_pos into simulation
    for i = 1:6
        res = vrep.simxSetJointTargetPosition(id, h.ur5Joints(i),...
            next_pos(i),...
            vrep.simx_opmode_oneshot);
        vrchk(vrep, res, true);
    end
    cur_pos = next_pos;
    gst = get_ur5_forward_kinematics(cur_pos,6)*gst0;
    % get gbt = inv(gsb) * gst
    gbt = gsb\gst;
    % take the logarithm of gbt to get ksi_hat
    ksi_hat = real(logm(gbt));
    % transform ksi_hat to ksi
    ksi = unskew(ksi_hat);

end

toc















