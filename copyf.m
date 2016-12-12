function hnew = copyf(id, vrep, g, h, hrel)
% The function duplicates the frame indicated by h, and places the new 
% frame at the pose indicated by g

% INPUT DEFINITIONS
% id: created via id = vrep.simxStart('127.0.0.1', 19997, true, true, 2000, 5)
% vrep: the object that contains all the vrep methods, created via vrep=remApi('remoteApi')
% g: a 4*4 homogeneous transformation matrix
% h: the handle to a frame object, e.g. handles.Frame0
% hrel: (optional) handle of the object relative to which you will move the frame.

% if there is no input for hrel, make it -1, which is to the world frame 
if exist('hrel','var') == 0
    fprintf('Frame is moved relative to V-REP absolute coordinates\n');
    hrel = -1;
end

% copy the initial frame
 	[res, hnew] = vrep.simxCopyPasteObjects(id, h, vrep.simx_opmode_blocking);
    vrchk(vrep, res);
    
% move the frame to the desired position & orientation
    movef(id, vrep, g, hnew, hrel);
    
end