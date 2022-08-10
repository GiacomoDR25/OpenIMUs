% quat2helic Transforms a quaternion to its axis-angle representation
%     [axis, angle] = quat2helic(q) returns the angle of rotation and axis
%     of rotation of quaternion q. q will be normalized prior to transformation.

function [axis, angle] = quat2helic(q)
    q = quat_normalize(q);
    
    angle = 2*acos(q(1));
    axis = [q(2) q(3) q(4)] ./ sqrt(1-q(1)^2);
end