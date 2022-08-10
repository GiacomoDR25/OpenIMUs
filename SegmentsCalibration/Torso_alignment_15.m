% TORSO ALIGNMENT
close all
%% MEDIO-LATERAL AXIS-Y

% Hypothesis:
% Principal axis of rotation during the sit-to-stand corresponds to the
% medio-lateral Y-axis [0 1 0] in the GLOBAL SYSTEM (X: forward, Y:lateral, Z:up)

% Get acceleration, angular velocity, quaternion for the sit-to-stand in GS and LS
t_Q = DSt.torso.Q(DSt.sit_range, :);% sit-to-stand quaternion range 
AngVel_GS = DSt.torso.g_GS(DSt.sit_range, :);% get angular velocity GS
AngVel_SS = DSt.torso.g(DSt.sit_range, :);% get angular velocity LS

figure;
subplot(2,1,1);plot(AngVel_SS(2:end, :));legend('X_{LS}' ,'Y_{LS}' ,'Z_{LS}');
title('Sit-to-Stand - Torso sensor - Local System');grid on;ylabel('angVel (rad/s)');
subplot(2,1,2);plot(AngVel_GS(2:end, :));legend('X_{GS}', 'Y_{GS}', 'Z_{GS}')
title('Sit-to-Stand - Torso sensor - Global System (not aligned with Y-Pelvis-axis)');grid on;ylabel('angVel (rad/s)');


%% Compute main axis of rotation in GS
% Low pass filter angular velocity to remove noise
AngVel = filtfilt3D(bLow, aLow, AngVel_GS(2:end, :));
AngVelNorm = sqrt(sum(AngVel.^2,2));

% To improve estimation accuracy we want to remove all instants of very
% low rotation speeds (since this low rotation speed could by in any direction)
% The angular velocity threshold was fixed at 30 deg/sec
angVelThres = deg2rad(25);
% figure;plot(AngVel(AngVelNorm>angVelThres,:));

% Compute the PCA to find the principal axis of rotation
principalAxes = pca(AngVel(AngVelNorm>angVelThres,:));

% Find the rotation that aligns the principal axis to the Y-axis (medio-lateral axis of the OS model)
[rotationAxis, rotationAngle] = vec2helic(principalAxes(:,1), [0 1 0]);
% axang = rotm2axang(principalAxes);

% figure;
% dr = HelperDrawRotation;
% dr.draw3DOrientation(gca, rotationAxis, rad2deg(rotationAngle));
% xlabel('X');ylabel('Y');zlabel('Z')
% axis equal;

% Transforms helical rotation (axis + angle) into a quaternion/Rot Max
q_axisRot = helic2quat(rotationAxis, rotationAngle);

rotmat_axisRot = quat2rotm(q_axisRot);% MATLAB function
rotmat_axisRot2 = quat2matrix(q_axisRot);% manual function

e = eulerd(quaternion(q_axisRot), 'XYZ', 'frame');% euler angles of the rotation
q_axisRot_t = quaternion([0 0 e(3)], 'eulerd', 'XYZ', 'frame');% only Z-angle so the gravity axis is kept
R_axisRot_t = quat2rotm(q_axisRot_t);% rotation matrix;

% p_Q_align1 = quaternion(q_axisRot).* p_Q; %sit-to-stand align with global system
t_Q_alignZ = quaternion(q_axisRot_t).* t_Q; %sit-to-stand align

AngVel_GS_alignZ = angvel(t_Q_alignZ, 1/fs, 'frame'); % get angular velocity GS
AngVel_GS_alignZ_f = filtfilt3D(bLow, aLow, AngVel_GS_alignZ(2:end, :));

figure;
subplot(1,2,1);plot(AngVel_GS_alignZ_f);legend('X_{GS}', 'Y_{GS}', 'Z_{GS}')
title('Sit-to-Stand - Torso sensor - Global System (aligned - Y_{rotZ})');grid on;ylabel('angVel (rad/s)');

%% LUMBAR BENDING and ROTATION OFFSET - SIT_TO_STAND AXIS (functional movement)
% Verify PCA-axis after rotation
AngVelNormZ = sqrt(sum(AngVel_GS_alignZ_f.^2,2));
% To improve estimation accuracy we want to remove all instants of very
% low rotation speeds (since this low rotation speed could by in any direction)
% The angular velocity threshold was fixed at 30 deg/sec
% Compute the PCA to find the principal axis of rotation
principalAxesZ = pca(AngVel_GS_alignZ_f(AngVelNormZ>angVelThres,:));


subplot(1,2,2); quiver3(0, 0, 0, principalAxesZ(1,1), principalAxesZ(2,1), principalAxesZ(3,1), 'LineWidth', 2);
title('Torso Rotation Axis during Sit-to-Stand');axis equal;xlabel('X');ylabel('Y');zlabel('Z');
ax = rad2deg(atan2(sqrt(principalAxesZ(2,1)^2+principalAxesZ(3,1)^2), principalAxesZ(1,1)));
ay = rad2deg(atan2(sqrt(principalAxesZ(3,1)^2+principalAxesZ(1,1)^2), principalAxesZ(2,1)));
az = rad2deg(atan2(sqrt(principalAxesZ(1,1)^2+principalAxesZ(2,1)^2), principalAxesZ(3,1)));

lumbar_bending = ax;
lumbar_rotation = az;
if lumbar_bending >=80
    lumbar_bending = 90-ax;
end
if lumbar_rotation >=80
    lumbar_rotation = az-90;
end
if lumbar_rotation > 20 || lumbar_rotation < -20
    lumbar_rotation = 0;
end

%% LUMBAR EXTENSION  - TORSO ROTATION AXIS (functional movement)
% if j < notW_n
    % Hypothesis:
    % Principal axis of rotation during the sit-to-stand corresponds to the
    % medio-lateral Y-axis [0 1 0] in the GLOBAL SYSTEM (X: forward, Y:lateral, Z:up)

    % Get acceleration, angular velocity, quaternion for the sit-to-stand in GS and LS
    t_Q = DSt.pelvis.Q(DSt.trunk_rot_range, :);% sit-to-stand quaternion range 
    AngVel_GS = DSt.pelvis.g_GS(DSt.trunk_rot_range, :);% get angular velocity GS
    AngVel_SS = DSt.pelvis.g(DSt.trunk_rot_range, :);% get angular velocity LS

    figure;
    subplot(1,3,1);plot(AngVel_SS(2:end, :));legend('X_{LS}' ,'Y_{LS}' ,'Z_{LS}');
    title('Trunk rotation - Torso sensor - Local System');grid on;ylabel('angVel (rad/s)');
    subplot(1,3,2);plot(AngVel_GS(2:end, :));legend('X_{GS}', 'Y_{GS}', 'Z_{GS}')
    title('Trunk rotation - Torso sensor - Global System (not aligned with Y-Pelvis-axis)');grid on;ylabel('angVel (rad/s)');

    % Compute main axis of rotation in GS
    % Low pass filter angular velocity to remove noise
    AngVel = filtfilt3D(bLow, aLow, AngVel_GS(2:end, :));
    AngVelNorm = sqrt(sum(AngVel.^2,2));

    % To improve estimation accuracy we want to remove all instants of very
    % low rotation speeds (since this low rotation speed could by in any direction)
    % The angular velocity threshold was fixed at 30 deg/sec
    angVelThres = deg2rad(20);
    % figure;plot(AngVel(AngVelNorm>angVelThres,:));

    % Compute the PCA to find the principal axis of rotation
    principalAxes = pca(AngVel(AngVelNorm>angVelThres,:));
    subplot(1,3,3);
    quiver3(0, 0, 0, principalAxes(1,1), principalAxes(2,1), principalAxes(3,1), 'LineWidth', 2);
    title('Torso Rotation Axis during Trunk rotation');axis equal;xlabel('X');ylabel('Y');zlabel('Z');

    [~, rotationAngle] = vec2helic(principalAxes(:,1), [0 0 1]);
    lumbar_extension = -rad2deg(rotationAngle);


% end
%% Estimate Global Acceleration - TORSO

DSt.torso.Q_GS = q_axisRot_t.* DSt.torso.Q;

DSt.torso.aZ = DSt.torso.a*R_axisRot_t;


static0 = mean(DSt.torso.a(DSt.standing_range,:)/GravityValue);
[rotationAxis0, rotationAngle0] = vec2helic([static0(1) static0(2) static0(3)], gravityVector);
gravityRotMatrix0 = quat2matrix(helic2quat(rotationAxis0, rotationAngle0));

staticZ = mean(DSt.torso.aZ(DSt.standing_range,:)/GravityValue);
[rotationAxisZ, rotationAngleZ] = vec2helic([staticZ(1) staticZ(2) staticZ(3)], gravityVector);
gravityRotMatrixZ = quat2matrix(helic2quat(rotationAxisZ, rotationAngleZ));

DSt.torso.aZ_GS = DSt.torso.aZ * gravityRotMatrixZ;
DSt.torso.a_GS = DSt.torso.a * gravityRotMatrix0;
DSt.torso.Q_GS_xyz =   DSt.torso.Q_GS*quaternion(rotm2quat(gravityRotMatrixZ));

% staticZ_GS = mean(DSt.pelvis.aZ_GS(DSt.standing_range,:)/GravityValue);
% figure;
% quiver3(0, 0, 0, staticZ_GS(1), staticZ_GS(2), staticZ_GS(3), 'LineWidth', 2);axis equal;
% [~, rotationAngle] = vec2helic(staticZ_GS, [0 0 -1]);
% pelvis_tilt = pelvis_tilt0 - abs(rad2deg(rotationAngle));
% 
% 
% 
% 
figure;
subplot(1,3,1);plot(DSt.torso.a(DSt.standing_range, :), 'LineWidth', 2);
title('Standing - Torso sensor - Local System 0');grid on;ylabel('acc (m/s^2)');legend('X_{LS0}', 'Y_{LS0}', 'Z_{LS0}');
subplot(1,3,2);plot(DSt.torso.aZ(DSt.standing_range, :), 'LineWidth', 2);
title('Standing - Torso sensor - Local System Rotated');grid on;ylabel('acc (m/s^2)');legend('X_{LS}', 'Y_{LS}', 'Z_{LS}');
subplot(1,3,3);plot(DSt.torso.aZ_GS(DSt.standing_range, :), 'LineWidth', 2);
title('Standing - Torso sensor - Global System - Aligned');grid on;ylabel('acc (m/s^2)');legend('X_{GS}', 'Y_{GS}', 'Z_{GS}');
%%
if lumbar_extension > 0 && pelvis_tilt > 0 
    lumbar_extension = - lumbar_extension;
end
if lumbar_extension < 0 && pelvis_tilt < 0 
    lumbar_extension = - lumbar_extension;
end

disp(['lumbar_extension: ', num2str(lumbar_extension), char(13)...
    'lumbar_bending: ', num2str(lumbar_bending), char(13)...
    'lumbar_rotation: ', num2str(lumbar_rotation)])
%% VIEWER
% viewer = HelperOrientationViewer;
% 
% for ii=1
%     viewer(p_Q_align1(ii,:));
%     pause(0.01);
% end
% % 
% % %%
% viewer = HelperOrientationViewer;
% 
% for ii=1
%     viewer(p_Q(ii,:));
%     pause(0.01);
% end