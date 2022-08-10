% matrix_2quat Transforms a rotation matrix into a quaternion
%    q = matrix_2quat(rotMatrix) returns the quaternion representing the
%    rotation matrix rotMatrix. The rotation matrix has to be normalized.
function q = matrix_2quat(rotMatrix)
    q = zeros(1,4);
    tr = trace(rotMatrix);
   
    if tr>0
        s = 0.5/sqrt(tr+1);
        q(1) = 0.25/s;
        q(2) = (rotMatrix(3,2)-rotMatrix(2,3))*s;
        q(3) = (rotMatrix(1,3)-rotMatrix(3,1))*s;
        q(4) = (rotMatrix(2,1)-rotMatrix(1,2))*s;
    else
        if rotMatrix(1,1)>rotMatrix(2,2) && rotMatrix(1,1)>rotMatrix(3,3)
            s = 2*sqrt(1 + rotMatrix(1,1) - rotMatrix(2,2) - rotMatrix(3,3));
            q(1) = (rotMatrix(3,2) - rotMatrix(2,3)) / s;
            q(2) = 0.25*s;
            q(3) = (rotMatrix(1,2) + rotMatrix(2,1)) / s;
            q(4) = (rotMatrix(1,3) + rotMatrix(3,1)) / s;
        elseif rotMatrix(2,2)>rotMatrix(3,3)
            s = 2*sqrt(1 + rotMatrix(2,2) - rotMatrix(1,1) - rotMatrix(3,3));
            q(1) = (rotMatrix(1,3) - rotMatrix(3,1)) / s;
            q(2) = (rotMatrix(1,2) + rotMatrix(2,1)) / s;
            q(3) = 0.25*s;
            q(4) = (rotMatrix(2,3) + rotMatrix(3,2)) / s;            
        else
            s = 2*sqrt(1 + rotMatrix(3,3) - rotMatrix(1,1) - rotMatrix(2,2));
            q(1) = (rotMatrix(2,1) - rotMatrix(1,2)) / s;
            q(2) = (rotMatrix(1,3) + rotMatrix(3,1)) / s;
            q(3) = (rotMatrix(2,3) + rotMatrix(3,2)) / s;
            q(4) = 0.25*s;    
        end
    end
end

