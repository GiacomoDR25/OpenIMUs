% PELVIS ALIGNMENT
close all
%% MEDIO-LATERAL AXIS-Y

% Hypothesis:
% Principal axis of rotation during the sit-to-stand corresponds to the
% medio-lateral Y-axis [0 1 0] in the GLOBAL SYSTEM (X: forward, Y:lateral, Z:up)

% Get acceleration, angular velocity, quaternion for the sit-to-stand in GS and LS
p_Q = DSt.pelvis.Q(DSt.sit_range, :);% sit-to-stand quaternion range 
AngVel_GS = DSt.pelvis.g_GS(DSt.sit_range, :);% get angular velocity GS
AngVel_SS = DSt.pelvis.g(DSt.sit_range, :);% get angular velocity LS

figure;
subplot(2,1,1);plot(AngVel_SS(2:end, :));legend('X_{LS}' ,'Y_{LS}' ,'Z_{LS}');
title('Sit-to-Stand - Pelvis sensor - Local System');grid on;ylabel('angVel (rad/s)');
subplot(2,1,2);plot(AngVel_GS(2:end, :));legend('X_{GS}', 'Y_{GS}', 'Z_{GS}')
title('Sit-to-Stand - Pelvis sensor - Global System (not aligned with Y-Pelvis-axis)');grid on;ylabel('angVel (rad/s)');


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
q_axisRot_p = quaternion([0 0 e(3)], 'eulerd', 'XYZ', 'frame');% only Z-angle so the gravity axis is kept
R_axisRot_p = quat2rotm(q_axisRot_p);% rotation matrix;

% p_Q_align1 = quaternion(q_axisRot).* p_Q; %sit-to-stand align with global system
p_Q_alignZ = quaternion(q_axisRot_p).* p_Q; %sit-to-stand align

AngVel_GS_alignZ = angvel(p_Q_alignZ, 1/fs, 'frame'); % get angular velocity GS
AngVel_GS_alignZ_f = filtfilt3D(bLow, aLow, AngVel_GS_alignZ(2:end, :));
[~, locs] = findpeaks(abs(AngVel_GS_alignZ_f(:,2)), 'minpeakheight', 0.75*std(AngVel_GS_alignZ_f(:,2)));

if AngVel_GS_alignZ_f(locs(1), 2) < 0
    R_axisRot_p = rotz(pi) * R_axisRot_p;
end

figure;plot(AngVel_GS_alignZ_f(:,2));legend('Y_{GS}');
title('Sit-to-Stand - pelvis sensor - Global System (aligned - Y_{rotZ})');grid on;ylabel('angVel (rad/s)');

%% PELVIS LIST and ROTATION OFFSET - SIT_TO_STAND AXIS
% Verify PCA-axis after rotation
AngVelNormZ = sqrt(sum(AngVel_GS_alignZ_f.^2,2));
% To improve estimation accuracy we want to remove all instants of very
% low rotation speeds (since this low rotation speed could by in any direction)
% The angular velocity threshold was fixed at 30 deg/sec
% Compute the PCA to find the principal axis of rotation
principalAxesZ = pca(AngVel_GS_alignZ_f(AngVelNormZ>angVelThres,:));

figure;
quiver3(0, 0, 0, principalAxesZ(1,1), principalAxesZ(2,1), principalAxesZ(3,1), 'LineWidth', 2);
title('Pelvis Rotation Axis during Sit-to-Stand');axis equal;xlabel('X');ylabel('Y');zlabel('Z');
ax = rad2deg(atan2(sqrt(principalAxesZ(2,1)^2+principalAxesZ(3,1)^2), principalAxesZ(1,1)));
ay = rad2deg(atan2(sqrt(principalAxesZ(3,1)^2+principalAxesZ(1,1)^2), principalAxesZ(2,1)));
az = rad2deg(atan2(sqrt(principalAxesZ(1,1)^2+principalAxesZ(2,1)^2), principalAxesZ(3,1)));

pelvis_list = ax;
pelvis_rotation = az;
if pelvis_list >=80
    pelvis_list = 90-ax;
end
if pelvis_rotation >=80
    pelvis_rotation = az-90;
end
if pelvis_rotation > 20 || pelvis_rotation < -20
    pelvis_rotation = 0;
end

%% PELVIS TILT OFFSET - TRUNK ROTATION AXIS
% if j < notW_n
    % Hypothesis:
    % Principal axis of rotation during the sit-to-stand corresponds to the
    % medio-lateral Y-axis [0 1 0] in the GLOBAL SYSTEM (X: forward, Y:lateral, Z:up)

    % Get acceleration, angular velocity, quaternion for the sit-to-stand in GS and LS
    p_Q = DSt.pelvis.Q(DSt.trunk_rot_range, :);% sit-to-stand quaternion range 
    AngVel_GS = DSt.pelvis.g_GS(DSt.trunk_rot_range, :);% get angular velocity GS
    AngVel_SS = DSt.pelvis.g(DSt.trunk_rot_range, :);% get angular velocity LS

    figure;
    subplot(2,1,1);plot(AngVel_SS(2:end, :));legend('X_{LS}' ,'Y_{LS}' ,'Z_{LS}');
    title('Trunk rotation - Pelvis sensor - Local System');grid on;ylabel('angVel (rad/s)');
    subplot(2,1,2);plot(AngVel_GS(2:end, :));legend('X_{GS}', 'Y_{GS}', 'Z_{GS}')
    title('Trunk rotation - Pelvis sensor - Global System (not aligned with Y-Pelvis-axis)');grid on;ylabel('angVel (rad/s)');

    % Compute main axis of rotation in GS
    % Low pass filter angular velocity to remove noise
    AngVel = filtfilt3D(bLow, aLow, AngVel_GS(2:end, :));
    AngVelNorm = sqrt(sum(AngVel.^2,2));

    % To improve estimation accuracy we want to remove all instants of very
    % low rotation speeds (since this low rotation speed could by in any direction)
    % The angular velocity threshold was fixed at 30 deg/sec
    angVelThres = deg2rad(20);
    figure;plot(AngVel(AngVelNorm>angVelThres,:));

    % Compute the PCA to find the principal axis of rotation
    principalAxes = pca(AngVel(AngVelNorm>angVelThres,:));
    figure;
    quiver3(0, 0, 0, principalAxes(1,1), principalAxes(2,1), principalAxes(3,1), 'LineWidth', 2);hold on;
    quiver3(0, 0, 0, 1, 0, 0, 'r','LineWidth', 2);
    quiver3(0, 0, 0, 0, 1, 0, 'g','LineWidth', 2);
    quiver3(0, 0, 0, 0, 0, 1, 'b','LineWidth', 2);
    title('Pelvis Rotation Axis during Trunk rotation');axis equal;xlabel('X');ylabel('Y');zlabel('Z');
    ax = rad2deg(atan2(sqrt(principalAxes(2,1)^2+principalAxes(3,1)^2), principalAxes(1,1)));% X-axis angle
    ay = rad2deg(atan2(sqrt(principalAxes(3,1)^2+principalAxes(1,1)^2), principalAxes(2,1)));% Y-axis angle
    az = rad2deg(atan2(sqrt(principalAxes(1,1)^2+principalAxes(2,1)^2), principalAxes(3,1)));% Z-axis angle
    
    [rotationAxis, rotationAngle] = vec2helic(principalAxes(:,1), gravityVector);
        
    if ax > 90
        pelvis_tilt0 = -rad2deg(rotationAngle);
    end

% end
%% Estimate Global Acceleration - PELVIS

DSt.pelvis.Q_GS = q_axisRot_p.* DSt.pelvis.Q;
R_axisRot_0 = quat2rotm(conj(quaternion(mean(compact(DSt.pelvis.Q(DSt.standing_range))))));
DSt.pelvis.a0 = DSt.pelvis.a*R_axisRot_0;
DSt.pelvis.aZ = DSt.pelvis.a0*R_axisRot_p*inv(R_axisRot_0);

staticZ = mean(DSt.pelvis.aZ(DSt.standing_range,:)/GravityValue);
figure;
quiver3(0, 0, 0, staticZ(1), staticZ(2), staticZ(3), 'LineWidth', 2);hold on;
quiver3(0, 0, 0, 1, 0, 0, 'r','LineWidth', 2);
quiver3(0, 0, 0, 0, 1, 0, 'g','LineWidth', 2);
quiver3(0, 0, 0, 0, 0, 1, 'b','LineWidth', 2);
title('     ');axis equal;xlabel('X');ylabel('Y');zlabel('Z');

[rotationAxisZ, rotationAngleZ] = vec2helic([staticZ(1) staticZ(2) staticZ(3)], gravityVector);
gravityRotMatrixZ = quat2matrix(helic2quat(rotationAxisZ, rotationAngleZ));

figure;
subplot(1,2,1);plot(DSt.pelvis.a(DSt.standing_range, :)/GravityValue, 'LineWidth', 2);
title('Standing - Pelvis sensor - Local System 0');grid on;ylabel('acc (m/s^2)');legend('X_{LS_0}', 'Y_{LS_0}', 'Z_{LS_0}');
axis([0 DSt.standing_range(end) -1 1]);
subplot(1,2,2);plot(DSt.pelvis.aZ(DSt.standing_range, :)/GravityValue, 'LineWidth', 2);
title('Standing - Pelvis sensor - Local System Aligned');grid on;ylabel('acc (m/s^2)');legend('X_{LS}', 'Y_{LS}', 'Z_{LS}');
axis([0 DSt.standing_range(end) -1 1]);

       %% Correction during upright
    
    % The movements may not have been perfectly executed: refine
    % calibration based on the upright posture
        
    % Constraint 1: zero abuction
    
      
    % Get the flexion angle (rotation around X axis)
    [a, abduction] = vec2helic([0 staticZ(2), staticZ(3)], gravityVector);
%     abductionCorrection = quat2matrix(helic2quat(a, abduction));
       
%     fprintf('    %s abduction correction %1.2fdeg\n', 'Pelvis', abduction/pi*180);
    
    pelvis_list = rad2deg(abduction);        
    
    % Constraint 2: zero flexion
    % --------------------------
     
    % Get the flexion angle (rotation around Y axis)

    [a, flexion] = vec2helic([staticZ(1) 0 staticZ(3)], gravityVector);
%     flexionCorrection = quat2matrix(helic2quat(a, flexion));
       
%     fprintf('    %s flexion correction %1.2fdeg\n', 'Pelvis', flexion/pi*180);
    
    pelvis_tilt = rad2deg(flexion);
    if pelvis_tilt > 45
        pelvis_tilt = 90- pelvis_tilt;
    end

   
%%
disp(['pelvis_tilt: ', num2str(pelvis_tilt), char(13)...
    'pelvis_list: ', num2str(pelvis_list), char(13)...
    'pelvis_rotation: ', num2str(pelvis_rotation)])
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