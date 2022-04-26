%%  CREATE IK .MOT

%% IMU Inverse Kinematics - CALIBRATED FILES (WALKING)
clc
for j = 5:9
% IMU registration
close all;
OutPath = pwd;
if j<10
    filename = ['IMU_00', num2str(j), '_P1'];
else
    filename = ['IMU_0', num2str(j), '_P1'];
end
modelFileName_IK = [ OutPath,'\S0', num2str(s),'_allMarkers_allIMU_calibrated.osim'];
visualizeTracking = false;
orientationsFileName_IK = [filename '_orientations_trial.sto'];
startTime = 0;
endTime = 100;
resultsDirectory = 'IKResults';
IK_xml = 'myIMUIK_Setup.xml';
OpenSense_folder = OutPath;

OpenSense_OrientationTracking_fx(modelFileName_IK, orientationsFileName_IK, ...
    startTime, endTime, resultsDirectory, visualizeTracking, IK_xml, OpenSense_folder)
end
beep;

%% IMU Inverse Kinematics - RANDOM MOVEMENTS WITH NO ALIGNMENTS
clc;
for j = 1:19
visualizeTracking = false;
if j<10
    filename = ['Session1_00', num2str(j)];
else
    filename = ['Session1_0', num2str(j)];
end
% IMU registration
orientationsFileName_IK = [filename '_orientations.sto'];
startTime = 0;
endTime = 100;
resultsDirectory = 'IKResults';
IK_xml = 'myIMUIK_Setup.xml';
OpenSense_folder = OutPath;

OpenSense_OrientationTracking_fx(modelFileName_IK, orientationsFileName_IK, ...
    startTime, endTime, resultsDirectory, visualizeTracking, IK_xml, OpenSense_folder)
end
toc
beep;

%% CREATE MOVEMENT STO FILE FOR OTHER MOTIONS (RUNNING / STAIRS)
tic
for j = 19:21
    
    DSt = getfield(DS, ['d', num2str(j)]);

% IMU registration
close all;
OutPath = pwd;
    if j<10
        filename = ['IMU_00', num2str(j), '_P1'];
    else
        filename = ['IMU_0', num2str(j), '_P1'];
    end
fr = fs;
% Headers = {'torso_imu','pelvis_imu', 'femur_r_imu','tibia_r_imu','toes_r_imu','femur_l_imu','tibia_l_imu','toes_l_imu'};
Headers = {'torso_imu','pelvis_imu','humerus_r_imu','radius_r_imu','humerus_l_imu','radius_l_imu', 'femur_r_imu','tibia_r_imu','calcn_r_imu','femur_l_imu','tibia_l_imu','calcn_l_imu'};
nbodies =  length(Headers);
fr_cal = 10;
fr_mov = 1:length(DSt.time);
nfr = size(DSt.time, 1);

baseIMUName = 'pelvis_imu';Placer_xml = [OutPath, '\myIMUPlacer_Setup.xml'];
% modelFileName = [ OutPath, '\S05_allMarkers_allIMU.osim'];
modelFileName = [ OutPath, '\S04_allMarkers_allIMU.osim'];
% modelFileName_IK = [ OutPath,'\S01_allMarkers_allIMU_calibrated.osim'];
modelFileName_IK = [ OutPath,'\S04_allMarkers_allIMU_calib_Py.osim'];
visualizeTracking = false;
calibrate = false;
% calibrate = true;

%% FAVRE -----------------------------------------------------------------------------------------------------------------------------------------------------------
run Pelvis_alignment_15_notrunk_rot.m
all_same = "true";% rotation among Z of different angle based on PCA + sit-to-stand/walking

%init
angleLT = 0;
angleRT = 0;
angleLC = 0;
angleRC = 0;

% run IMU_Placer_2_IMUv3
run IMU_Placer_2_v8
% 
% % Orientation in the model with PCA
% run IMU_Placer_2_v7% calculate global orientation GS without Favre
%% IMU WITH MAG
close all
clc
% % visualization = "true";
visualization = "false";
% 
% init
DSt.tibia_l.Q_rot = DSt.tibia_l.Q_GS;
DSt.tibia_r.Q_rot = DSt.tibia_r.Q_GS;
DSt.calcn_l.Q_rot = DSt.calcn_l.Q_GS;
DSt.calcn_r.Q_rot = DSt.calcn_r.Q_GS;

% -------------------  CALCUALTE THE ANGLE OF EXTRA ROTATION BASED ON FAVRE
%LEFT TIBIA
disp('--------------------- tibia_l ---------------------')
[angleLT, DSt.femur_l.Q_rot, DSt.tibia_l.Q_rot] = ...
    favre_alignment_2(DSt.adduction_range_l, DSt.femur_l.g_SS, DSt.tibia_l.g_SS, ...
    DSt.femur_l.R_GS, DSt.tibia_l.R_GS, fs, visualization);
%RIGHT TIBIA
disp('--------------------- tibia_r ---------------------')
[angleRT, DSt.femur_r.Q_rot, DSt.tibia_r.Q_rot] =...
    favre_alignment_2(DSt.adduction_range_r, DSt.femur_r.g_SS, DSt.tibia_r.g_SS, ...
    DSt.femur_r.R_GS, DSt.tibia_r.R_GS, fs, visualization);

%-----------   UPDATE GS orientation in the model with PCA+FAVRE
qLT = quaternion([0 0 angleLT], 'eulerd', 'XYZ', 'frame');% left tibia
qRT = quaternion([0 0 angleRT], 'eulerd', 'XYZ', 'frame');% right tibia

% RIGHT TIBIA Rotation
DSt.tibia_r.Q_GS2 = (qRT).*DSt.tibia_r.Q_GS; %all the entire movement
DSt.tibia_r.R_GS2 = quat2rotm(DSt.tibia_r.Q_GS2);
DSt.tibia_r.g_SS2 = angvel(DSt.tibia_r.Q_GS2, 1/fs, 'point');% get angular velocity LS    
% LEFT TIBIA Rotation
DSt.tibia_l.Q_GS2 = (qLT).* DSt.tibia_l.Q_GS; %all the entire movement
DSt.tibia_l.R_GS2 = quat2rotm(DSt.tibia_l.Q_GS2);
DSt.tibia_l.g_SS2 = angvel(DSt.tibia_l.Q_GS2, 1/fs, 'point');% get angular velocity LS

%LEFT CALCANEUS
disp('--------------------- calcn_l ---------------------')
[angleLC, DSt.tibia_l.Q_rot, DSt.calcn_l.Q_rot] = ...
    favre_alignment_2(DSt.adduction_range_l, DSt.tibia_l.g_SS2, DSt.calcn_l.g_SS, ...
    DSt.tibia_l.R_GS2, DSt.calcn_l.R_GS, fs, visualization);
%RIGHT CALCANEUS
disp('--------------------- calcn_r ---------------------')
[angleRC, DSt.tibia_r.Q_rot, DSt.calcn_r.Q_rot] = ...
    favre_alignment_2(DSt.adduction_range_r, DSt.tibia_r.g_SS2, DSt.calcn_r.g_SS,...
    DSt.tibia_r.R_GS2, DSt.calcn_r.R_GS, fs, visualization);
 
qLC = quaternion([0 0 angleLC], 'eulerd', 'XYZ', 'frame');% left toes
qRC = quaternion([0 0 angleRC], 'eulerd', 'XYZ', 'frame');% right toes

% RIGHT CALCN Rotation
DSt.calcn_r.Q_GS2 = (qRC).* DSt.calcn_r.Q_GS; %all the entire movement
DSt.calcn_r.R_GS2 = quat2rotm(DSt.calcn_r.Q_GS2);
DSt.calcn_r.g_SS2 = angvel(DSt.calcn_r.Q_GS2, 1/fs, 'point');% get angular velocity LS
% LEFT CALCN Rotation
DSt.calcn_l.Q_GS2 = (qLC).* DSt.calcn_l.Q_GS; %all the entire movement
DSt.calcn_l.R_GS2 = quat2rotm(DSt.calcn_l.Q_GS2);
DSt.calcn_l.g_SS2 = angvel(DSt.calcn_l.Q_GS2, 1/fs, 'point');% get angular velocity LS

%% Create Matrix   - Xsens (Favre ONLY)

DSt.Data_Matrix_cal = [compact(DSt.torso.Q_GS(fr_cal)), compact(DSt.pelvis.Q_GS(fr_cal)), ...
    compact(DSt.humerus_r.Q_GS(fr_cal)), compact(DSt.radius_r.Q_GS(fr_cal))...
    compact(DSt.humerus_l.Q_GS(fr_cal)), compact(DSt.radius_l.Q_GS(fr_cal))...
    compact(DSt.femur_r.Q_GS(fr_cal)), compact(DSt.tibia_r.Q_GS2(fr_cal)), compact(DSt.calcn_r.Q_GS2(fr_cal)),...
    compact(DSt.femur_l.Q_GS(fr_cal)), compact(DSt.tibia_l.Q_GS2(fr_cal)), compact(DSt.calcn_l.Q_GS2(fr_cal))];

DSt.Data_Matrix = [compact(DSt.torso.Q_GS(fr_mov)), compact(DSt.pelvis.Q_GS(fr_mov)),...
    compact(DSt.humerus_r.Q_GS(fr_mov)), compact(DSt.radius_r.Q_GS(fr_mov))...
    compact(DSt.humerus_l.Q_GS(fr_mov)), compact(DSt.radius_l.Q_GS(fr_mov))...
    compact(DSt.femur_r.Q_GS(fr_mov)), compact(DSt.tibia_r.Q_GS2(fr_mov)), compact(DSt.calcn_r.Q_GS2(fr_mov)),...
    compact(DSt.femur_l.Q_GS(fr_mov)), compact(DSt.tibia_l.Q_GS2(fr_mov)), compact(DSt.calcn_l.Q_GS2(fr_mov))];


%% Create .STO file
Create_IMU_Storage(fullfile(OutPath, [filename '_orientations_cal.sto']), 1, nbodies, 0, DSt.Data_Matrix_cal, Headers);
Create_IMU_Storage(fullfile(OutPath, [filename '_orientations_trial.sto']), nfr, nbodies, DSt.time, DSt.Data_Matrix, Headers);
pause(1)


%% IMU Inverse Kinematics
clc;
% IMU registration
close all;
orientationsFileName_IK = [filename '_orientations_trial.sto'];
startTime = 0;
endTime = 100;
resultsDirectory = 'IKResults';
IK_xml = 'myIMUIK_Setup.xml';
OpenSense_folder = OutPath;

OpenSense_OrientationTracking_fx(modelFileName_IK, orientationsFileName_IK, ...
    startTime, endTime, resultsDirectory, visualizeTracking, IK_xml, OpenSense_folder)
end
toc
beep;


