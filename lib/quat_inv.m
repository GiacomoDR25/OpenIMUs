% quat_inv Inverses a quaternion
%    qinv = quat_inv(q) computes the inverse of quaternion q

function qInv = quat_inv(q)
    qConj = q .* [1 -1 -1 -1];    
    qInv = qConj ./ (q(1)^2+q(2)^2+q(3)^2+q(4)^2);
end