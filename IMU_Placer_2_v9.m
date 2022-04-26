%% -------------- Functional IMU PLACER ------------------

% Favre rotations

qLT = quaternion([0 0 angleLT], 'eulerd', 'XYZ', 'frame');% left tibia
qRT = quaternion([0 0 angleRT], 'eulerd', 'XYZ', 'frame');% right tibia
qLC = quaternion([0 0 angleLC], 'eulerd', 'XYZ', 'frame');% left toes
qRC = quaternion([0 0 angleRC], 'eulerd', 'XYZ', 'frame');% right toes

%% PCA Axis Heading of single Sensor
if all_same == "false"

    DSt.humerus_r.Q_GS = q_rotationsM(9).* DSt.humerus_r.Q; %all the entire movement
        
    DSt.radius_r.Q_GS = q_rotationsM(11).* DSt.radius_r.Q; %all the entire movement
        
    DSt.humerus_l.Q_GS = q_rotationsM(10).* DSt.humerus_l.Q; %all the entire movement
        
    DSt.radius_l.Q_GS = q_rotationsM(12).* DSt.radius_l.Q; %all the entire movement
    
    %TORSO
    DSt.torso.Q_GS = q_rotationsM(2).* DSt.torso.Q; %all the entire movement
    %PELVIS
    DSt.pelvis.Q_GS = q_rotationsM(1).* DSt.pelvis.Q; %all the entire movement
    % RIGHT FEMUR
    DSt.femur_r.Q_GS = q_rotationsM(3).* DSt.femur_r.Q; %all the entire movement
    DSt.femur_r.R_GS = quat2rotm(DSt.femur_r.Q_GS);
    DSt.femur_r.g_SS = angvel(DSt.femur_r.Q_GS, 1/fs, 'point');% get angular velocity LS
    % RIGHT TIBIA
    DSt.tibia_r.Q_GS = (qRT).*q_rotationsM(5).*DSt.tibia_r.Q; %all the entire movement
    DSt.tibia_r.R_GS = quat2rotm(DSt.tibia_r.Q_GS);
    DSt.tibia_r.g_SS = angvel(DSt.tibia_r.Q_GS, 1/fs, 'point');% get angular velocity LS    
    % RIGHT CALCANEUS
    DSt.calcn_r.Q_GS = (qRC).*q_rotationsM(7).* DSt.calcn_r.Q; %all the entire movement
    DSt.calcn_r.R_GS = quat2rotm(DSt.calcn_r.Q_GS);
    DSt.calcn_r.g_SS = angvel(DSt.calcn_r.Q_GS, 1/fs, 'point');% get angular velocity LS
    % LEFT FEMUR
    DSt.femur_l.Q_GS = q_rotationsM(4).* DSt.femur_l.Q; %all the entire movement
    DSt.femur_l.R_GS = quat2rotm(DSt.femur_l.Q_GS);
    DSt.femur_l.g_SS = angvel(DSt.femur_l.Q_GS, 1/fs, 'point');% get angular velocity LS
    % LEFT TIBIA
    DSt.tibia_l.Q_GS = (qLT).*q_rotationsM(6).* DSt.tibia_l.Q; %all the entire movement
    DSt.tibia_l.R_GS = quat2rotm(DSt.tibia_l.Q_GS);
    DSt.tibia_l.g_SS = angvel(DSt.tibia_l.Q_GS, 1/fs, 'point');% get angular velocity LS
    % LEFT CALCANEUS
    DSt.calcn_l.Q_GS = (qLC).*q_rotationsM(8).* DSt.calcn_l.Q; %all the entire movement
    DSt.calcn_l.R_GS = quat2rotm(DSt.calcn_l.Q_GS);
    DSt.calcn_l.g_SS = angvel(DSt.calcn_l.Q_GS, 1/fs, 'point');% get angular velocity LS
    
end
