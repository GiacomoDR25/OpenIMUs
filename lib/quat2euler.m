% quat_2euler Converts a quaternion into 3-2-1 Euler angles
%    euler = quat_2euler(q) converts quaternion q into the Euler angles
%    stored in the 3x1 vector euler

function euler = quat_2euler(q)
    euler(1) = atan2(2*(q(1)*q(2)+q(3)*q(4)), 1-2*(q(2)^2+q(3)^2));
    euler(2) = asin(2*(q(1)*q(3)-q(4)*q(2)));
    euler(3) = atan2(2*(q(1)*q(4)+q(2)*q(3)), 1-2*(q(3)^2+q(4)^2));
end