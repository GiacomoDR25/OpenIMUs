% HUMERUS L ALIGNMENT
close all
%% MEDIO-LATERAL AXIS-Y

% Hypothesis:
% Principal axis of rotation during the sit-to-stand corresponds to the
% medio-lateral Y-axis [0 1 0] in the GLOBAL SYSTEM (X: forward, Y:lateral, Z:up)

% Get acceleration, angular velocity, quaternion for the sit-to-stand in GS and LS
hl_Q = DSt.humerus_l.Q(DSt.walking_range, :);% sit-to-stand quaternion range 
AngVel_GS = DSt.humerus_l.g_GS(DSt.walking_range, :);% get angular velocity GS
AngVel_SS = DSt.humerus_l.g(DSt.walking_range, :);% get angular velocity LS

figure;
subplot(2,1,2);plot(AngVel_GS(2:end, :));legend('X_{GS}', 'Y_{GS}', 'Z_{GS}')
title('Humerus left sensor - Global System (not aligned Y-axis)');grid on;ylabel('angVel (rad/s)');
subplot(2,1,1);plot(AngVel_SS(2:end, :));legend('X_{LS}' ,'Y_{LS}' ,'Z_{LS}');
title('Humerus left sensor - Local System');grid on;ylabel('angVel (rad/s)');

%% Compute main axis of rotation in GS
% Low pass filter angular velocity to remove noise
AngVel = filtfilt3D(bNormal, aNormal, AngVel_GS(2:end, :));
AngVelNorm = sqrt(sum(AngVel.^2,2));

% To improve estimation accuracy we want to remove all instants of very
% low rotation speeds (since this low rotation speed could by in any direction)
% The angular velocity threshold was fixed at 50 deg/sec
angVelThres = deg2rad(50);

% figure;plot(AngVel(AngVelNorm>angVelThres,:))

% Compute the PCA to find the principal axis of rotation
principalAxes = pca(AngVel(AngVelNorm>angVelThres,:));

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
%%
e1 = eulerd(quaternion(q_axisRot), 'XYZ', 'frame');%get the angles of the axis
% disp(['rotAngle Z - humerus l = ' , num2str(e1(3)), ' deg']);

q_axisRot_hl = quaternion([0 0 e1(3)], 'eulerd', 'XYZ', 'frame');% get rotation quaternion among Z only

hl_Q_align1 = quaternion(q_axisRot).* hl_Q; 


AngVel_GS_align = angvel(hl_Q_align1, 1/fs, 'frame'); % get angular velocity GS (new orientation)
AngVel_GS_align_f = filtfilt3D(bNormal, aNormal, AngVel_GS_align(2:end, :));

AngVelNorm = sqrt(sum(AngVel_GS_align_f.^2,2));

figure;plot(AngVel_GS_align_f);legend('X_{GS}', 'Y_{GS}', 'Z_{GS}')
title('Humerus left sensor - Global System (Aligned with Y)');grid on;ylabel('angVel (rad/s)');

%% Estimate Global Acceleration 
DSt.humerus_l.Q_GS = quaternion(q_axisRot_hl).* DSt.humerus_l.Q;
static = median(DSt.humerus_l.a(DSt.standing_range,:)/GravityValue);
[rotationAxis, rotationAngle] = vec2helic([static(1) static(2) static(3)], gravityVector);
gravityRotMatrix = quat2matrix(helic2quat(rotationAxis, rotationAngle));

DSt.humerus_l.a_GS = DSt.humerus_l.a * gravityRotMatrix;
DSt.humerus_l.a_rot = DSt.humerus_l.a * quat2rotm(q_axisRot_hl);

% figure;plot(acc_GS); legend('x','y','z');
% viewer =   HelperOrientationViewer;
% viewer(DSt.humerus_l.Q_GS(DSt.standing_range));

figure;plot(DSt.humerus_l.a(DSt.standing_range,:));
hold on;plot(DSt.humerus_l.a_rot(DSt.standing_range,:)); legend('x','y','z','x1','y1','z1');

    
    
    
%% After rotation among Z)

hl_Q_alignZ = quaternion(q_axisRot_hl).* hl_Q;

AngVel_GS_alignZ = angvel(hl_Q_alignZ, 1/fs, 'frame'); % get angular velocity GS (new orientation)
AngVel_GS_alignZ_f = filtfilt3D(bNormal, aNormal, AngVel_GS_alignZ(2:end, :));
figure;plot(AngVel_GS_alignZ_f);legend('X_{GS}', 'Y_{GS}', 'Z_{GS}')
title('Humerus left sensor - Global System (Aligned with Y_{rotZonly})');grid on;ylabel('angVel (rad/s)');


AngVelNormZ = sqrt(sum(AngVel_GS_alignZ_f.^2,2));

% To improve estimation accuracy we want to remove all instants of very
% low rotation speeds (since this low rotation speed could by in any direction)
% The angular velocity threshold was fixed at 50 deg/sec


% figure;plot(AngVel(AngVelNorm>angVelThres,:))

% Compute the PCA to find the principal axis of rotation
principalAxesZ = pca(AngVel_GS_alignZ_f(AngVelNormZ>angVelThres,:));

figure;
quiver3(0, 0, 0, principalAxesZ(1,1), principalAxesZ(2,1), principalAxesZ(3,1));
title('Humerus Left Rotation Axis');axis equal;xlabel('X');ylabel('Y');zlabel('Z');
ax = rad2deg(atan2(sqrt(principalAxesZ(2,1)^2+principalAxesZ(3,1)^2), principalAxesZ(1,1)));
ay = rad2deg(atan2(sqrt(principalAxesZ(3,1)^2+principalAxesZ(1,1)^2), principalAxesZ(2,1)));
az = rad2deg(atan2(sqrt(principalAxesZ(1,1)^2+principalAxesZ(2,1)^2), principalAxesZ(3,1)));
%% VIEWER
% viewer = HelperOrientationViewer;
% for ii=1:size(hl_Q)
%     viewer(hl_Q(ii,:));
%     pause(0.01);
% end
% 
% 
% viewer = HelperOrientationViewer;
% for ii=1:size(hl_Q_align1)
%     viewer(hl_Q_align1(ii,:));
%     pause(0.01);
% end
% 
% 
% viewer = HelperOrientationViewer;
% for ii=1:size(hl_Q_alignZ)
%     viewer(hl_Q_alignZ(ii,:));
%     pause(0.01);
% end

%% COORD HIP ADD - FLE - ROT
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