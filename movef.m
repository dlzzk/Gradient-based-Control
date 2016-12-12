function movef(id, vrep, g, h, hrel)
% INPUT DEFINITIONS
% id: created via id = vrep.simxStart('127.0.0.1', 19997, true, true, 2000, 5)
% vrep: the object that contains all the vrep methods, created via vrep=remApi('remoteApi')
% g: a 4*4 homogeneous transformation matrix
% h: the handle to a frame object, e.g. handles.Frame0
% hrel: (optional) handle of the object relative to which you will move the frame.

% Extract rotation matrix from the input homogeneous transformation matrix
R = g(1:3,1:3);
% Get the rotate angles from the rotation matrix
rotate_angles = EULERZYXINV(R);
% Extract translation part from the input
trans = g(1:3,4);
 
% if there is no input for hrel, make it -1, which is to the world frame 
if exist('hrel') == 0
    fprintf('Frame is moved relative to V-REP absolute coordinates\n');
    hrel = -1;
end

% rotate the frame
[res]=vrep.simxSetObjectOrientation(id, ...
    h, hrel, rotate_angles, vrep.simx_opmode_oneshot);

% translate the frame
[res] = vrep.simxSetObjectPosition(id, ...
    h, hrel, trans, vrep.simx_opmode_oneshot);



end