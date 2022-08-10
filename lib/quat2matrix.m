% quat2matrix Converts a quaternion into 3x3 orientation matrix
%    R = quat2matrix(q) converts quaternion q into orientation matrix R

function R = quat2matrix(q)
    q = quat_normalize(q);
    
    R=zeros(3,3);
    R(1,1)=q(1)^2+q(2)^2-q(3)^2-q(4)^2;
    R(1,2)=2*(q(2)*q(3)-q(4)*q(1));
    R(1,3)=2*(q(2)*q(4)+q(3)*q(1));
 
    R(2,1)=2*(q(2)*q(3)+q(4)*q(1));
    R(2,2)=q(1)^2-q(2)^2+q(3)^2-q(4)^2;
    R(2,3)=2*(q(3)*q(4)-q(2)*q(1));
  
    R(3,1)=2*(q(2)*q(4)-q(3)*q(1));
    R(3,2)=2*(q(3)*q(4)+q(2)*q(1));
    R(3,3)=q(1)^2-q(2)^2-q(3)^2+q(4)^2;
end