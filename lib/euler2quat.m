% euler_2quat Converts 3-2-1 Euler angles into a quaternion
%    quat = euler_2quat(phi, theta, psi) converts 3-2-1 euler angles phi,
%    theta and psi (given in radians) into a quaternion. In biomechanics
%    phi is abduction-adduction (rotation around ant-post axis), theta is
%    internal-external rotation (rotation around vertical axis) and psi is
%    flexion-extension (rotation around medio-lateral axis)

function quat = euler_2quat(phi, theta, psi)
    quat(1) = cos(phi/2)*cos(theta/2)*cos(psi/2) + sin(phi/2)*sin(theta/2)*sin(psi/2);
    quat(2) = sin(phi/2)*cos(theta/2)*cos(psi/2) - cos(phi/2)*sin(theta/2)*sin(psi/2);
    quat(3) = cos(phi/2)*sin(theta/2)*cos(psi/2) + sin(phi/2)*cos(theta/2)*sin(psi/2);
    quat(4) = cos(phi/2)*cos(theta/2)*sin(psi/2) - sin(phi/2)*sin(theta/2)*cos(psi/2);
end