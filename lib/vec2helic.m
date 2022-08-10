% vec2helic Finds the rotation axis and angle between two vectors
%    [rotationAxis, rotationAngle] = vec2helic(v1,v2) computes the rotation
%    axis and rotation angle of the rotation that rotates vector v1 to vector v2

function [rotationAxis, rotationAngle] = vec2helic(v1,v2)

    % The rotation angle is the acos of dot-product between the two vectors
%     rotationAngle=acos(dot(v1,v2)/(norm(v1)*norm(v2)));
    v1 = v1(:)';
    v2 = v2(:);
    rotationAngle=acos((v1*v2)/(norm(v1)*norm(v2)));
    
    % Find the rotation axis (always normal to the two vectors v1 and v2)
    rotationAxis=cross(v1,v2);
    
    % Normalize the rotation axis
    rotationAxis=rotationAxis/norm(rotationAxis);
end