% quat_normalize Normalizes a quaternion
%    q = quat_normalize(q) normalizes the quaternion q (i.e. sets its norm
%    to 1)

function q = quat_normalize(q)
    q = q ./ sqrt(q(1)^2+q(2)^2+q(3)^2+q(4)^2);
end