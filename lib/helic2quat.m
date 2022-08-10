% helic2quat Transforms helical rotation into a quaternion
%    q = helic2quat(axis, angle) transforms the rotation by angle radians
%    around axis axis into a quaternion

function q = helic2quat(axis, angle)

   % Initialize the quaternion
   q = zeros(1,4);
    
   % Rotation angle
   q(1) = cos(angle/2);
   
   % Rotation axis
   axisNorm = norm(axis);
   if axisNorm~=0
       q(2:4) = axis./norm(axis) .* sin(angle/2);
   else
       q(2:4) = 0;
   end
   
   % Normalize
   q = quat_normalize(q);
end
