%% ---------- FEMUR RIGHT ALIGNMENT ----------
close all
%% MEDIO-LATERAL AXIS-Y

% Hypothesis:
% Principal axis of rotation during the sit-to-stand corresponds to the
% medio-lateral Y-axis [0 1 0] in the GLOBAL SYSTEM (X: forward, Y:lateral, Z:up)

% Get acceleration, angular velocity, quaternion for the sit-to-stand in GS and LS
fr_Q = DSt.femur_r.Q(DSt.sit_range, :);% sit-to-stand quaternion range 
AngVel_GS = DSt.femur_r.g_GS(DSt.sit_range, :);% get angular velocity GS
AngVel_SS = DSt.femur_r.g(DSt.sit_range, :);% get angular velocity LS

figure;
subplot(2,1,2);plot(AngVel_GS(2:end, :));legend('X_{GS}', 'Y_{GS}', 'Z_{GS}')
title('Sit-to-Stand - Femur R sensor - Global System (not aligned)');grid on;ylabel('angVel (rad/s)');
subplot(2,1,1);plot(AngVel_SS(2:end, :));legend('X_{LS}' ,'Y_{LS}' ,'Z_{LS}');
title('Sit-to-Stand - Femur R sensor - Local System');grid on;ylabel('angVel (rad/s)');


% Compute main axis of rotation in GS
% Low pass filter angular velocity to remove noise
% squatAngVel = filtfilt3D(bLow, aLow, squatAngVel_GS(2:end, :));
AngVel = filtfilt3D(bLow, aLow, AngVel_GS(2:end, :));
AngVelNorm = sqrt(sum(AngVel.^2,2));

% To improve estimation accuracy we want to remove all instants of very
% low rotation speeds (since this low rotation speed could by in any direction)
% The angular velocity threshold was fixed at 30 deg/sec
angVelThres = deg2rad(20);

% figure;plot(squatAngVel(squatAngVelNorm>angVelThres,:))

% Compute the PCA to find the principal axis of rotation
principalAxes = pca(AngVel(AngVelNorm>angVelThres,:));

% Find the rotation that aligns the principal axis to the Y-axis (medio-lateral axis of the OS model)
[rotationAxis, rotationAngle] = vec2helic(principalAxes(:,1), [0 1 0]);
% disp(['rotAngle = ' , num2str(rad2deg(rotationAngle)), newline,'rotAxis = [',...
%     num2str(rotationAxis), ']'])
% axang = rotm2axang(principalAxes);

% figure;
% dr = HelperDrawRotation;
% dr.draw3DOrientation(gca, rotationAxis, rad2deg(rotationAngle));
% xlabel('X');ylabel('Y');zlabel('Z')
% axis equal;

% Transforms helical rotation (axis + angle) into a quaternion/Rot Max
q_axisRot = helic2quat(rotationAxis, rotationAngle);


e = eulerd(quaternion(q_axisRot), 'XYZ', 'frame');
q_axisRot_fr = quaternion([0 0 e(3)], 'eulerd', 'XYZ', 'frame');
R_axisRot_fr = quat2rotm(q_axisRot_fr);
rotmat_axisRot = quat2rotm(q_axisRot);% MATLAB function
rotmat_axisRot2 = quat2matrix(q_axisRot);% manual function

fr_Q_align1 = quaternion(q_axisRot).* fr_Q; %sit-to-stand align
fr_Q_alignZ = quaternion(q_axisRot_fr).* fr_Q; %sit-to-stand align

AngVel_GS_alignZ = angvel(fr_Q_alignZ, 1/fs, 'frame'); % get angular velocity GS
AngVel_GS_alignZ_f = filtfilt3D(bLow, aLow, AngVel_GS_alignZ(2:end, :));
AngVel_GS_alignZ_f = AngVel_GS_alignZ_f(10:end-10,:);

figure;
plot(AngVel_GS_alignZ_f);legend('X_{GS}', 'Y_{GS}', 'Z_{GS}');
title('Sit-to-Stand - Femur right sensor - Global System (aligned - Y-axis)');
grid on;ylabel('angVel (rad/s)');

%% HIP ADDUCTION and ROTATION offset - Walking (functional movement)
fr_Q = DSt.femur_r.Q(DSt.sit_range, :);% sit-to-stand quaternion range 
AngVel_GS = DSt.femur_r.g_GS(DSt.sit_range, :);% get angular velocity GS
AngVel_SS = DSt.femur_r.g(DSt.sit_range, :);% get angular velocity LS

figure;
subplot(1,3,1);plot(AngVel_GS(2:end, :));legend('X_{GS}', 'Y_{GS}', 'Z_{GS}')
title('Walking - Femur left sensor - Global System (not aligned)');grid on;ylabel('angVel (rad/s)');
subplot(1,3,2);plot(AngVel_SS(2:end, :));legend('X_{LS}' ,'Y_{LS}' ,'Z_{LS}');
title('Walking - Femur left sensor - Local System');grid on;ylabel('angVel (rad/s)');


% Compute main axis of rotation in GS
% Low pass filter angular velocity to remove noise
% squatAngVel = filtfilt3D(bLow, aLow, squatAngVel_GS(2:end, :));
AngVel = filtfilt3D(bNormal, aNormal, AngVel_GS(2:end, :));
AngVelNorm = sqrt(sum(AngVel.^2,2));

% To improve estimation accuracy we want to remove all instants of very
% low rotation speeds (since this low rotation speed could by in any direction)
% The angular velocity threshold was fixed at 30 deg/sec
angVelThres = deg2rad(50);

% figure;plot(squatAngVel(squatAngVelNorm>angVelThres,:))

% Compute the PCA to find the principal axis of rotation
principalAxes = pca(AngVel(AngVelNorm>angVelThres,:));

% Find the rotation that aligns the principal axis to the Y-axis (medio-lateral axis of the OS model)
[rotationAxis, rotationAngle] = vec2helic(principalAxes(:,1), [0 1 0]);
q_axisRot = helic2quat(rotationAxis, rotationAngle);
e = eulerd(quaternion(q_axisRot), 'XYZ', 'frame');
q_axisRotZ = quaternion([0 0 e(3)], 'eulerd', 'XYZ', 'frame');
R_axisRotZ = quat2rotm(q_axisRotZ);

fr_Q_alignZ = quaternion(q_axisRotZ).* fr_Q; %sit-to-stand align.* fr_Q;
% fr_Q_alignZ = quaternion(q_axisRot_fr).* fr_Q; %sit-to-stand align

AngVel_GS_alignZ = angvel(fr_Q_alignZ, 1/fs, 'frame'); % get angular velocity GS
AngVel_GS_alignZ_f = filtfilt3D(bLow, aLow, AngVel_GS_alignZ(2:end, :));
AngVel_GS_alignZ_f = AngVel_GS_alignZ_f(10:end-10,:);
AngVelNormZ = sqrt(sum(AngVel_GS_alignZ_f.^2,2));
% To improve estimation accuracy we want to remove all instants of very
% low rotation speeds (since this low rotation speed could by in any direction)
% The angular velocity threshold was fixed at 30 deg/sec
% Compute the PCA to find the principal axis of rotation
% figure;plot(AngVel_GS_alignZ_f(AngVelNormZ>angVelThres,:))
angVelThres = deg2rad(20);
principalAxesZ = pca(AngVel_GS_alignZ_f(AngVelNormZ>angVelThres,:));

subplot(1,3,3);
figure;
quiver3(0, 0, 0, principalAxesZ(1,1), principalAxesZ(2,1), principalAxesZ(3,1), 'LineWidth', 2);hold on;
quiver3(0, 0, 0, 1, 0, 0, 'r','LineWidth', 2);
quiver3(0, 0, 0, 0, 1, 0, 'g','LineWidth', 2);
quiver3(0, 0, 0, 0, 0, 1, 'b','LineWidth', 2);
title('Hip Right Walking Rotation Axis');axis equal;xlabel('X');ylabel('Y');zlabel('Z');
ax = rad2deg(atan2(sqrt(principalAxesZ(2,1)^2+principalAxesZ(3,1)^2), principalAxesZ(1,1)));% X-axis angle
ay = rad2deg(atan2(sqrt(principalAxesZ(3,1)^2+principalAxesZ(1,1)^2), principalAxesZ(2,1)));% Y-axis angle
az = rad2deg(atan2(sqrt(principalAxesZ(1,1)^2+principalAxesZ(2,1)^2), principalAxesZ(3,1)));% Z-axis angle
hip_rotation_r = -ay;
if az > 90
    hip_adduction_r = 90 - az; 
else
    hip_adduction_r = - (90 - az);
end

%% HIP FLEXION offset - Hip adduction (functional movement)
% Hypothesis:
% Principal axis of rotation during the sit-to-stand corresponds to the
% medio-lateral Y-axis [0 1 0] in the GLOBAL SYSTEM (X: forward, Y:lateral, Z:up)

% Get acceleration, angular velocity, quaternion for the sit-to-stand in GS and LS
fr_Q = DSt.femur_r.Q(DSt.adduction_range_r, :);% sit-to-stand quaternion range 
AngVel_GS = DSt.femur_r.g_GS(DSt.adduction_range_r, :);% get angular velocity GS
AngVel_SS = DSt.femur_r.g(DSt.adduction_range_r, :);% get angular velocity LS

figure;
subplot(2,1,2);plot(AngVel_GS(2:end, :));legend('X_{GS}', 'Y_{GS}', 'Z_{GS}')
title('Hip Abd-Adduction - right sensor - Global System (not aligned)');grid on;ylabel('angVel (rad/s)');
subplot(2,1,1);plot(AngVel_SS(2:end, :));legend('X_{LS}' ,'Y_{LS}' ,'Z_{LS}');
title('Hip Abd-Adduction - right left sensor - Local System');grid on;ylabel('angVel (rad/s)');

% Low pass filter angular velocity to remove noise
AngVel = filtfilt3D(bLow, aLow, AngVel_GS(2:end, :));
AngVelNorm = sqrt(sum(AngVel.^2,2));

% To improve estimation accuracy we want to remove all instants of very
% low rotation speeds (since this low rotation speed could by in any direction)
% The angular velocity threshold was fixed at 30 deg/sec
angVelThres = deg2rad(20);
% figure;plot(squatAngVel(squatAngVelNorm>angVelThres,:))
% Compute the PCA to find the principal axis of rotation
principalAxes = pca(AngVel(AngVelNorm>angVelThres,:));
% Find the rotation that aligns the principal axis to the X-axis (forward-back axis of the OS model)
[rotationAxis, rotationAngle] = vec2helic(principalAxes(:,1), [1 0 0]);

% figure;
% dr = HelperDrawRotation;
% dr.draw3DOrientation(gca, rotationAxis, rad2deg(rotationAngle));
% xlabel('X');ylabel('Y');zlabel('Z')
% axis equal;

% Transforms helical rotation (axis + angle) into a quaternion/Rot Max
q_axisRot = helic2quat(rotationAxis, rotationAngle);

e = eulerd(quaternion(q_axisRot), 'XYZ', 'frame');

q_axisRotZ = quaternion([0 0 e(3)], 'eulerd', 'XYZ', 'frame');
R_axisRotZ = quat2rotm(q_axisRotZ);
fr_Q_alignZ = quaternion(q_axisRotZ).* fr_Q; %sit-to-stand align
AngVel_GS_alignZ = angvel(fr_Q_alignZ, 1/fs, 'frame'); % get angular velocity GS
AngVel_GS_alignZ_f = filtfilt3D(bLow, aLow, AngVel_GS_alignZ(2:end, :));

figure;
subplot(2,1,1);plot(AngVel_GS_alignZ_f);legend('X_{GS}', 'Y_{GS}', 'Z_{GS}')
title('Hip AA - right sensor - Global System (aligned - Y_{rotZ})');grid on;ylabel('angVel (rad/s)');

% Verify PCA-axis after rotation
AngVelNormZ = sqrt(sum(AngVel_GS_alignZ_f.^2,2));
% To improve estimation accuracy we want to remove all instants of very
% low rotation speeds (since this low rotation speed could by in any direction)
% The angular velocity threshold was fixed at 30 deg/sec
% Compute the PCA to find the principal axis of rotation
principalAxesZ = pca(AngVel_GS_alignZ_f(AngVelNormZ>angVelThres,:));

subplot(2,1,2);
quiver3(0, 0, 0, principalAxesZ(1,1), principalAxesZ(2,1), principalAxesZ(3,1), 'LineWidth', 2);hold on;
quiver3(0, 0, 0, 1, 0, 0, 'r','LineWidth', 2);
quiver3(0, 0, 0, 0, 1, 0, 'g','LineWidth', 2);
quiver3(0, 0, 0, 0, 0, 1, 'b','LineWidth', 2);
title('Hip AA Rotation Axis');axis equal;xlabel('X');ylabel('Y');zlabel('Z');

ax = rad2deg(atan2(sqrt(principalAxesZ(2,1)^2+principalAxesZ(3,1)^2), principalAxesZ(1,1)));% X-axis angle
ay = rad2deg(atan2(sqrt(principalAxesZ(3,1)^2+principalAxesZ(1,1)^2), principalAxesZ(2,1)));% Y-axis angle
az = rad2deg(atan2(sqrt(principalAxesZ(1,1)^2+principalAxesZ(2,1)^2), principalAxesZ(3,1)));% Z-axis angle

hip_flexion_r = ax + pelvis_tilt;
%%

if hip_flexion_r > 0 && pelvis_tilt > 0 
    hip_flexion_r = - hip_flexion_r;
end
if hip_flexion_r < 0 && pelvis_tilt < 0 
    hip_flexion_r = - hip_flexion_r;
end

disp(['hip_flexion_r: ', num2str(hip_flexion_r), char(13),...
    'hip_adduction_r: ', num2str(hip_adduction_r), char(13),...
    'hip_rotation_r: ', num2str(hip_rotation_r)]);

%% Estimate Global Acceleration - FEMUR R

DSt.femur_r.Q_GS = q_axisRot_fr.* DSt.femur_r.Q;

DSt.femur_r.aZ = DSt.femur_r.a*R_axisRot_fr;


static0 = mean(DSt.femur_r.a(DSt.standing_range,:)/GravityValue);
[rotationAxis0, rotationAngle0] = vec2helic([static0(1) static0(2) static0(3)], gravityVector);
gravityRotMatrix0 = quat2matrix(helic2quat(rotationAxis0, rotationAngle0));

staticZ = mean(DSt.femur_r.aZ(DSt.standing_range,:)/GravityValue);
[rotationAxisZ, rotationAngleZ] = vec2helic([staticZ(1) staticZ(2) staticZ(3)], gravityVector);
gravityRotMatrixZ = quat2matrix(helic2quat(rotationAxisZ, rotationAngleZ));

DSt.femur_r.aZ_GS = DSt.femur_r.aZ * gravityRotMatrixZ;
DSt.femur_r.a_GS = DSt.femur_r.a * gravityRotMatrix0;
DSt.femur_r.Q_GS_xyz =   DSt.femur_r.Q_GS*quaternion(rotm2quat(gravityRotMatrixZ));

figure;
subplot(1,3,1);plot(DSt.femur_r.a(DSt.standing_range, :), 'LineWidth', 2);
title('Standing - femur_r sensor - Local System 0');grid on;ylabel('acc (m/s^2)');legend('X_{LS}', 'Y_{LS}', 'Z_{LS}');
subplot(1,3,2);plot(DSt.femur_r.aZ(DSt.standing_range, :), 'LineWidth', 2);
title('Standing - femur_r sensor - Local System Rotated');grid on;ylabel('acc (m/s^2)');legend('X_{LS}', 'Y_{LS}', 'Z_{LS}');
subplot(1,3,3);plot(DSt.femur_r.aZ_GS(DSt.standing_range, :), 'LineWidth', 2);
title('Standing - femur_r sensor - Global System - Aligned');grid on;ylabel('acc (m/s^2)');legend('X_{GS}', 'Y_{GS}', 'Z_{GS}');
%% VIEWER
% viewer = HelperOrientationViewer;
% 
% for ii=1
%     viewer(fl_Q_align1(ii,:));
%     pause(0.01);
% end
% % 
% % %%
% viewer = HelperOrientationViewer;
% 
% for ii=1
%     viewer(fl_Q(ii,:));
%     pause(0.01);
% end


