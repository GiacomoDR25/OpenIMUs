% RADIUS R ALIGNMENT
close all
%% MEDIO-LATERAL AXIS-Y

% Hypothesis:
% Principal axis of rotation during the sit-to-stand corresponds to the
% medio-lateral Y-axis [0 1 0] in the GLOBAL SYSTEM (X: forward, Y:lateral, Z:up)

% Get acceleration, angular velocity, quaternion for the sit-to-stand in GS and LS
rr_Q = DSt.radius_r.Q(DSt.walking_range, :);% sit-to-stand quaternion range 
squatAngVel_GS = DSt.radius_r.g_GS(DSt.walking_range, :);% get angular velocity GS
squatAngVel_SS = DSt.radius_r.g(DSt.walking_range, :);% get angular velocity LS

figure;
subplot(2,1,2);plot(squatAngVel_GS(2:end, :));legend('X_{GS}', 'Y_{GS}', 'Z_{GS}')
title('Humerus right sensor - Global System (not aligned Y-axis)');grid on;ylabel('angVel (rad/s)');
subplot(2,1,1);plot(squatAngVel_SS(2:end, :));legend('X_{LS}' ,'Y_{LS}' ,'Z_{LS}');
title('Humerus right sensor - Local System');grid on;ylabel('angVel (rad/s)');

%% Compute main axis of rotation in GS
% Low pass filter angular velocity to remove noise
squatAngVel = filtfilt3D(bLow, aLow, squatAngVel_GS(2:end, :));
squatAngVelNorm = sqrt(sum(squatAngVel.^2,2));

% To improve estimation accuracy we want to remove all instants of very
% low rotation speeds (since this low rotation speed could by in any direction)
% The angular velocity threshold was fixed at 30 deg/sec
angVelThres = deg2rad(20);

% figure;plot(squatAngVel(squatAngVelNorm>angVelThres,:))

% Compute the PCA to find the principal axis of rotation
principalAxes = pca(squatAngVel(squatAngVelNorm>angVelThres,:));

% Find the rotation that aligns the principal axis to the Y-axis (medio-lateral axis of the OS model)
[rotationAxis, rotationAngle] = vec2helic(principalAxes(:,1), [0 1 0]);
% axang = rotm2axang(principalAxes);
% disp(['rotAngle = ' , num2str(rad2deg(rotationAngle)), newline,'rotAxis = [',...
%     num2str(rotationAxis), ']'])

figure;
dr = HelperDrawRotation;
dr.draw3DOrientation(gca, rotationAxis, rad2deg(rotationAngle));
xlabel('X');ylabel('Y');zlabel('Z')
axis equal;

% Transforms helical rotation (axis + angle) into a quaternion/Rot Max
q_axisRot = helic2quat(rotationAxis, rotationAngle);

rotmat_axisRot = quat2rotm(q_axisRot);% MATLAB function
rotmat_axisRot2 = quat2matrix(q_axisRot);% manual function

e = eulerd(quaternion(q_axisRot), 'XYZ', 'frame');
q_axisRot_rr = quaternion([0 0 e(3)], 'eulerd', 'XYZ', 'frame');

rr_Q_align1 = quaternion(q_axisRot).* rr_Q; 

squatAngVel_GS_align = angvel(rr_Q_align1, 1/fs, 'frame'); % get angular velocity GS
squatAngVel_GS_align_f = filtfilt3D(bLow, aLow, squatAngVel_GS_align(2:end, :));

squatAngVelNorm = sqrt(sum(squatAngVel_GS_align_f.^2,2));

figure;plot(squatAngVel_GS_align_f);legend('X_{GS}', 'Y_{GS}', 'Z_{GS}')
title('Walking - Femur right sensor - Global System (Aligned with Y)');grid on;ylabel('angVel (rad/s)');

% disp(['rotAngle Z - humerus r = ' , num2str(e(3)), ' deg']);

%%
viewer = HelperOrientationViewer;
% for ii=1:size(rr_Q_align1)
%     viewer(rr_Q_align1(ii,:));
%     pause(0.01);
% end

% %% COORD HIP ADD - FLE - ROT
% 
% % Hypothesis:
% % Principal axis of rotation during the sit-to-stand corresponds to the
% % medio-lateral Y-axis [0 1 0] in the GLOBAL SYSTEM (X: forward, Y:lateral, Z:up)
% 
% % Get acceleration, angular velocity, quaternion for the sit-to-stand in GS and LS
% hr_Q = DS.femur_r.Q(DS.adduction_range1r, :);% sit-to-stand quaternion range 
% squatAngVel_GS = DS.femur_r.g_GS(DS.adduction_range1r, :);% get angular velocity GS
% squatAngVel_SS = DS.femur_r.g(DS.adduction_range1r, :);% get angular velocity LS
% 
% % Low pass filter angular velocity to remove noise
% squatAngVel = filtfilt3D(bLow, aLow, squatAngVel_GS(2:end, :));
% squatAngVelNorm = sqrt(sum(squatAngVel.^2,2));
% 
% % To improve estimation accuracy we want to remove all instants of very
% % low rotation speeds (since this low rotation speed could by in any direction)
% % The angular velocity threshold was fixed at 30 deg/sec
% angVelThres = deg2rad(10);
% 
% figure;plot(squatAngVel(squatAngVelNorm>angVelThres,:))
% 
% % Compute the PCA to find the principal axis of rotation
% principalAxes = pca(squatAngVel(squatAngVelNorm>angVelThres,:));
% 
% % Find the rotation that aligns the principal axis to the X-axis (forward-back axis of the OS model)
% [rotationAxis, rotationAngle] = vec2helic(principalAxes(:,1), [1 0 0]);
% % disp(['rotAngle = ' , num2str(rad2deg(rotationAngle)), newline,'rotAxis = [',...
% %     num2str(rotationAxis), ']'])
% % axang = rotm2axang(principalAxes);
% 
% figure;
% dr = HelperDrawRotation;
% dr.draw3DOrientation(gca, rotationAxis, rad2deg(rotationAngle));
% xlabel('X');ylabel('Y');zlabel('Z')
% axis equal;
% 
% % Transforms helical rotation (axis + angle) into a quaternion/Rot Max
% q_axisRot = helic2quat(rotationAxis, rotationAngle);
% 
% e = eulerd(quaternion(q_axisRot), 'XYZ', 'frame');
% hip_flexion_r = e(2)
% 
% %% Adduction
% hr_Q = DS.femur_r.Q(DS.sit_range, :);% sit-to-stand quaternion range 
% squatAngVel_GS = DS.femur_r.g_GS(DS.sit_range, :);% get angular velocity GS
% squatAngVel_SS = DS.femur_r.g(DS.sit_range, :);% get angular velocity LS
% 
% % Low pass filter angular velocity to remove noise
% squatAngVel = filtfilt3D(bLow, aLow, squatAngVel_GS(2:end, :));
% squatAngVelNorm = sqrt(sum(squatAngVel.^2,2));
% 
% % To improve estimation accuracy we want to remove all instants of very
% % low rotation speeds (since this low rotation speed could by in any direction)
% % The angular velocity threshold was fixed at 30 deg/sec
% angVelThres = deg2rad(10);
% 
% figure;plot(squatAngVel(squatAngVelNorm>angVelThres,:))
% 
% % Compute the PCA to find the principal axis of rotation
% principalAxes = pca(squatAngVel(squatAngVelNorm>angVelThres,:));
% 
% % Find the rotation that aligns the principal axis to the Y-axis (forward-back axis of the OS model)
% [rotationAxis, rotationAngle] = vec2helic(principalAxes(:,1), [0 1 0]);
% % disp(['rotAngle = ' , num2str(rad2deg(rotationAngle)), newline,'rotAxis = [',...
% %     num2str(rotationAxis), ']'])
% % axang = rotm2axang(principalAxes);
% 
% figure;
% dr = HelperDrawRotation;
% dr.draw3DOrientation(gca, rotationAxis, rad2deg(rotationAngle));
% xlabel('X');ylabel('Y');zlabel('Z')
% axis equal;
% 
% % Transforms helical rotation (axis + angle) into a quaternion/Rot Max
% q_axisRot = helic2quat(rotationAxis, rotationAngle);
% 
% e = eulerd(quaternion(q_axisRot), 'XYZ', 'frame');
% hip_adduction_r = -e(2)
% hip_rotation_r = -e(1)