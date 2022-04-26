%% CALCN LEFT ALIGNMENT
%% Heading correction
close all
% Hypothesis:
% Principal axis of rotation during the HIP adduction corresponds to the
% anterio-posterior X-axis [1 0 0] in the GLOBAL SYSTEM (X: forward, Y:lateral, Z:up)

% Get acceleration, angular velocity, quaternion for the sit-to-stand in GS and LS
cal_Q = DSt.calcn_l.Q(DSt.walking_range, :);
AngVel_GS = DSt.calcn_l.g_GS(DSt.walking_range, :); % get angular velocity GS
AngVel_SS = DSt.calcn_l.g(DSt.walking_range, :);% get angular velocity LS

figure;
subplot(2,1,2);plot(AngVel_GS(2:end, :));legend('X_{GS}', 'Y_{GS}', 'Z_{GS}')
title('Walking - foot L sensor - Global System (not aligned Y-axis)');grid on;ylabel('angVel (rad/s)');
subplot(2,1,1);plot(AngVel_SS(2:end, :));legend('X_{LS}' ,'Y_{LS}' ,'Z_{LS}');
title('Walking - foot L sensor - Local System');grid on;ylabel('angVel (rad/s)');

%% Compute main axis of rotation in GS
% Low pass filter angular velocity to remove noise
AngVel = filtfilt3D(bNormal, aNormal, AngVel_GS(2:end, :));% walking/running
AngVelNorm = sqrt(sum(AngVel.^2,2));

% To improve estimation accuracy we want to remove all instants of very
% low rotation speeds (since this low rotation speed could by in any direction)
% The angular velocity threshold was fixed at 30 deg/sec
angVelThres = deg2rad(100);

% figure;plot(squatAngVel(squatAngVelNorm>angVelThres,:))
% Compute the PCA to find the principal axis of rotation
principalAxes = pca(AngVel(AngVelNorm>angVelThres,:));

% Find the rotation that aligns the principal axis to the Y-axis (sit-to-stand axis of the OS model)
[rotationAxis, rotationAngle] = vec2helic(principalAxes(:,1), [0 1 0]);
% axang = rotm2axang(principalAxes);

% figure;
% dr = HelperDrawRotation;
% dr.draw3DOrientation(gca, rotationAxis, rad2deg(rotationAngle));
% axis equal

% Transforms helical rotation (axis + angle) into a quaternion/Rot Max
q_axisRot = helic2quat(rotationAxis, rotationAngle);
rotmat_axisRot = quat2rotm(q_axisRot);% MATLAB function
rotmat_axisRot2 = quat2matrix(q_axisRot);% manual function

e = eulerd(quaternion(q_axisRot), 'XYZ', 'frame');
q_axisRot_cal = quaternion([0 0 e(3)], 'eulerd', 'XYZ', 'frame');


cal_Q_align1 = quaternion(q_axisRot).* cal_Q; %sit-to-stand align
cal_Q_alignZ = quaternion(q_axisRot_cal).* cal_Q; %sit-to-stand align

AngVel_GS_alignZ = angvel(cal_Q_alignZ, 1/fs, 'frame'); % get angular velocity GS
AngVel_GS_alignZ_f = filtfilt3D(bLow, aLow, AngVel_GS_alignZ(2:end, :));
AngVel_GS_alignZ_f = AngVel_GS_alignZ_f(10:end-10,:);
figure;plot(AngVel_GS_alignZ_f);legend('X_{GS}', 'Y_{GS}', 'Z_{GS}')
title('Walking - foot L sensor - Global System (aligned X-axis)');grid on;ylabel('angVel (rad/s)');

%% Verify PCA-axis after rotation ---- ADDUCTION from WALKING
AngVelNormZ = sqrt(sum(AngVel_GS_alignZ_f.^2,2));
% To improve estimation accuracy we want to remove all instants of very
% low rotation speeds (since this low rotation speed could by in any direction)
% The angular velocity threshold was fixed at 30 deg/sec
% Compute the PCA to find the principal axis of rotation
% figure;plot(AngVel_GS_alignZ_f(AngVelNormZ>angVelThres,:))
principalAxesZ = pca(AngVel_GS_alignZ_f(AngVelNormZ>angVelThres,:));

figure;
quiver3(0, 0, 0, principalAxesZ(1,1), principalAxesZ(2,1), principalAxesZ(3,1), 'LineWidth', 2);hold on;
quiver3(0, 0, 0, 1, 0, 0, 'r','LineWidth', 2);
quiver3(0, 0, 0, 0, 1, 0, 'g','LineWidth', 2);
quiver3(0, 0, 0, 0, 0, 1, 'b','LineWidth', 2);
title('Ankle L Flexion Rotation Axis');axis equal;xlabel('X');ylabel('Y');zlabel('Z');
ax = rad2deg(atan2(sqrt(principalAxesZ(2,1)^2+principalAxesZ(3,1)^2), principalAxesZ(1,1)));
ay = rad2deg(atan2(sqrt(principalAxesZ(3,1)^2+principalAxesZ(1,1)^2), principalAxesZ(2,1)));
az = rad2deg(atan2(sqrt(principalAxesZ(1,1)^2+principalAxesZ(2,1)^2), principalAxesZ(3,1)));

if az > 90
    az = 90-az;
else
    az = - (az - 90);
end

subtalar_angle_l = sqrt(az^2+ay^2);

%% Verify PCA-axis after rotation ---- FLEXION from HIP ADDUCTION
% Hypothesis:
% Principal axis of rotation during the sit-to-stand corresponds to the
% medio-lateral Y-axis [0 1 0] in the GLOBAL SYSTEM (X: forward, Y:lateral, Z:up)

% Get acceleration, angular velocity, quaternion for the sit-to-stand in GS and LS
cal_Q = DSt.calcn_l.Q(DSt.adduction_range_l, :);% sit-to-stand quaternion range 
AngVel_GS = DSt.calcn_l.g_GS(DSt.adduction_range_l, :);% get angular velocity GS
AngVel_SS = DSt.calcn_l.g(DSt.adduction_range_l, :);% get angular velocity LS

figure;
subplot(2,1,2);plot(AngVel_GS(2:end, :));legend('X_{GS}', 'Y_{GS}', 'Z_{GS}')
title('Hip Abd-Adduction - L sensor - Global System (not aligned)');grid on;ylabel('angVel (rad/s)');
subplot(2,1,1);plot(AngVel_SS(2:end, :));legend('X_{LS}' ,'Y_{LS}' ,'Z_{LS}');
title('Hip Abd-Adduction - L sensor - Local System');grid on;ylabel('angVel (rad/s)');

%% Low pass filter angular velocity to remove noise
AngVel = filtfilt3D(bLow, aLow, AngVel_GS(2:end, :));
AngVelNorm = sqrt(sum(AngVel.^2,2));

% To improve estimation accuracy we want to remove all instants of very
% low rotation speeds (since this low rotation speed could by in any direction)
% The angular velocity threshold was fixed at 30 deg/sec
angVelThres = deg2rad(10);
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

% p_Q_align1 = quaternion(q_axisRot).* fl_Q; %sit-to-stand align with global system


cal_Q_alignZ = quaternion(q_axisRotZ).* cal_Q; %sit-to-stand align
AngVel_GS_alignZ = angvel(cal_Q_alignZ, 1/fs, 'frame'); % get angular velocity GS
AngVel_GS_alignZ_f = filtfilt3D(bLow, aLow, AngVel_GS_alignZ(2:end, :));

figure;plot(AngVel_GS_alignZ_f);legend('X_{GS}', 'Y_{GS}', 'Z_{GS}')
title('Hip AA - foot L sensor - Global System (aligned - X_{rotZ})');grid on;ylabel('angVel (rad/s)');

%% Verify PCA-axis after rotation
AngVelNormZ = sqrt(sum(AngVel_GS_alignZ_f.^2,2));
% To improve estimation accuracy we want to remove all instants of very
% low rotation speeds (since this low rotation speed could by in any direction)
% The angular velocity threshold was fixed at 30 deg/sec
% Compute the PCA to find the principal axis of rotation
principalAxesZ = pca(AngVel_GS_alignZ_f(AngVelNormZ>angVelThres,:));

figure;
quiver3(0, 0, 0, principalAxesZ(1,1), principalAxesZ(2,1), principalAxesZ(3,1), 'LineWidth', 2);hold on;
quiver3(0, 0, 0, 1, 0, 0, 'r','LineWidth', 2);
quiver3(0, 0, 0, 0, 1, 0, 'g','LineWidth', 2);
quiver3(0, 0, 0, 0, 0, 1, 'b','LineWidth', 2);
title('Hip AA Rotation Axis');axis equal;xlabel('X');ylabel('Y');zlabel('Z');
ax = rad2deg(atan2(sqrt(principalAxesZ(2,1)^2+principalAxesZ(3,1)^2), principalAxesZ(1,1)));
ay = rad2deg(atan2(sqrt(principalAxesZ(3,1)^2+principalAxesZ(1,1)^2), principalAxesZ(2,1)));
az = rad2deg(atan2(sqrt(principalAxesZ(1,1)^2+principalAxesZ(2,1)^2), principalAxesZ(3,1)));

ankle_angle_l = -knee_angle_l - ax;
%%

if ankle_angle_l <0
    ankle_angle_l = -ankle_angle_l;
end
subtalar_angle_l = 0;

disp(['ankle_angle_l: ', num2str(ankle_angle_l), char(13),...
        'subtalar_angle_l: ', num2str(subtalar_angle_l)]);
    