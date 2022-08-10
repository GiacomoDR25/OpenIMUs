% quat_multiply Multiplies to quaternions
%    qout = quat_multiply(q1, q2) multiplies q1 with q2. q1 and q2 must be
%    a 1x4 line vector representing a quaternion.

function qout = quat_multiply(q1, q2)

    % Make sure it's always a line vector
    q1 = [q1(1) q1(2) q1(3) q1(4)];
    
    % Prepare q2 for the multiplication
    Q = [q2(1) q2(2) q2(3) q2(4); ...
         -q2(2) q2(1) -q2(4) q2(3); ...
         -q2(3) q2(4) q2(1) -q2(2); ...
         -q2(4) -q2(3) q2(2) q2(1)];
     
    % Perform the multiplication
    qout = q1*Q;    
end