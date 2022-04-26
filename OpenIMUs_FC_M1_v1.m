%% FUCTIONAL CALIBRATION
clear;
close all
clc;

%% LOADING
LL_only = false;
s=2; % subject number
p=1;% pipeline 1
load(['Data_Struct_S0', num2str(s) ]);
%% Lowpass filter parameters
GravityValue = 9.81;
gravityVector = [0 0 1];
fs=60;
[bNormal,aNormal] = butter(2, 1.2465*6/fs*2, 'low');  % cutoff freq. 6 Hz (gait)
[bLow,aLow] = butter(2, 1.2465*1/fs*2, 'low');        % cutoff freq. 1 Hz (functional mov)
qrotx = quaternion([-90, 0, 0],'eulerd','XYZ','frame');% Xsens-to-OpenSim
Coord_off  = zeros(22,15); 
q_rotations  = zeros(12,4,3);% 5 walking / 12 joints
q_calibration  = zeros(12,4,3);% 5 walking / 12 joints
alignment_angles = zeros(4,3);% 4 angle / 5 walking
n=3; % 1st walking trial -1
%% PIPELINE CALIBRATION P1
tic
for j = 4:6
    
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
if LL_only
    Headers = {'torso_imu','pelvis_imu', 'femur_r_imu','tibia_r_imu','calcn_r_imu','femur_l_imu','tibia_l_imu','calcn_l_imu'};
else
    Headers = {'torso_imu','pelvis_imu','humerus_r_imu','radius_r_imu','humerus_l_imu','radius_l_imu', 'femur_r_imu','tibia_r_imu','calcn_r_imu','femur_l_imu','tibia_l_imu','calcn_l_imu'};
end
nbodies =  length(Headers);
fr_cal = 10;
fr_mov = 1:length(DSt.time);
nfr = size(DSt.time, 1);

baseIMUName = 'pelvis_imu';
Placer_xml = [OutPath, '\myIMUPlacer_Setup.xml'];

if s < 10
    modelFileName = [ OutPath, '\S0',num2str(s),'_allMarkers_allIMU.osim'];
    modelFileName_IK = [ OutPath,'\S0',num2str(s),'_allMarkers_allIMU_calibrated.osim'];
else
    modelFileName = [ OutPath, '\S',num2str(s),'_allMarkers_allIMU.osim'];
    modelFileName_IK = [ OutPath,'\S',num2str(s),'_allMarkers_allIMU_calibrated.osim'];
end
visualizeTracking = false;
calibrate = false;
% calibrate = true;

%% FAVRE  ONLY----------------------------------------------------------------------------------------------------------------------------------------------------------
run Pelvis_alignment_15.m
all_same = "true";% rotation among Z of different angle based on PCA + sit-to-stand/walking

%init
angleLT = 0;
angleRT = 0;
angleLC = 0;
angleRC = 0;

% run IMU_Placer_2_IMUv3

if LL_only
    run OpenIMUs_Placer_LL_v8.m
else
    run OpenIMUs_Placer_v8.m
end
    
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
    OpenIMUs_AngAlign_v3(DSt.adduction_range_l, DSt.femur_l.g_SS, DSt.tibia_l.g_SS, ...
    DSt.femur_l.R_GS, DSt.tibia_l.R_GS, fs, visualization);
%RIGHT TIBIA
disp('--------------------- tibia_r ---------------------')
[angleRT, DSt.femur_r.Q_rot, DSt.tibia_r.Q_rot] =...
    OpenIMUs_AngAlign_v3(DSt.adduction_range_r, DSt.femur_r.g_SS, DSt.tibia_r.g_SS, ...
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
    OpenIMUs_AngAlign_v3(DSt.adduction_range_l, DSt.tibia_l.g_SS2, DSt.calcn_l.g_SS, ...
    DSt.tibia_l.R_GS2, DSt.calcn_l.R_GS, fs, visualization);
%RIGHT CALCANEUS
disp('--------------------- calcn_r ---------------------')
[angleRC, DSt.tibia_r.Q_rot, DSt.calcn_r.Q_rot] = ...
    OpenIMUs_AngAlign_v3(DSt.adduction_range_r, DSt.tibia_r.g_SS2, DSt.calcn_r.g_SS,...
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

alignment_angles(1, j-n) = angleLT;
alignment_angles(2, j-n) = angleRT;
alignment_angles(3, j-n) = angleLC;
alignment_angles(4, j-n) = angleRC;

%% LOWERLIMBS ONLY
if LL_only 
    DSt.humerus_r.Q_GS = 0*DSt.torso.Q_GS;
    DSt.humerus_l.Q_GS = 0*DSt.torso.Q_GS;
    DSt.radius_r.Q_GS = 0*DSt.torso.Q_GS;
    DSt.radius_l.Q_GS = 0*DSt.torso.Q_GS;
end
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

%% Q calibration
    q_calibration(1,:, j-n) = compact(qrotx*DSt.torso.Q_GS(fr_cal));
    q_calibration(2,:, j-n) = compact(qrotx*DSt.pelvis.Q_GS(fr_cal));
    q_calibration(3,:, j-n) = compact(qrotx*DSt.femur_r.Q_GS(fr_cal));
    q_calibration(4,:, j-n) = compact(qrotx*DSt.femur_l.Q_GS(fr_cal));
    q_calibration(5,:, j-n) = compact(qrotx*DSt.tibia_r.Q_GS2(fr_cal));
    q_calibration(6,:, j-n) = compact(qrotx*DSt.tibia_l.Q_GS2(fr_cal));
    q_calibration(7,:, j-n) = compact(qrotx*DSt.calcn_r.Q_GS2(fr_cal));
    q_calibration(8,:, j-n) = compact(qrotx*DSt.calcn_l.Q_GS2(fr_cal));
    q_calibration(9,:, j-n) = compact(qrotx*DSt.humerus_r.Q_GS(fr_cal));
    q_calibration(10,:, j-n) = compact(qrotx*DSt.humerus_l.Q_GS(fr_cal));
    q_calibration(11,:, j-n) = compact(qrotx*DSt.radius_r.Q_GS(fr_cal));
    q_calibration(12,:, j-n) = compact(qrotx*DSt.radius_l.Q_GS(fr_cal));
    
end
toc
beep;


%% MEAN VALUES
q_rotationsM = quaternion(mean(q_rotations, 3));
q_calibrationM = quaternion(median(q_calibration, 3));
align_anglesM = mean(alignment_angles,2);% only for error statistics

%% Calibrate Model
coord_estimation = false;
visualizeIMU = false;
if s<10
    modelFileName = [ OutPath, '\S0', num2str(s),'_allMarkers_allIMU.osim'];
else
    modelFileName = [ OutPath, '\S', num2str(s),'_allMarkers_allIMU.osim'];
end
run OpenIMUs_CE_v1.m

