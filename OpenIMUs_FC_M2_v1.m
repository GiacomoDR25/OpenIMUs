%% FUCTIONAL CALIBRATION
clear;
close all
clc;

%% LOADING
LL_only = false;
s=6;% subject
p=2;% pipeline
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
n=5; % 1st walking trial -1
%% PIPELINE CALIBRATION P2
tic
for j = 6:10
    
    DSt = getfield(DS, ['d', num2str(j)]);

% IMU registration
close all;
OutPath = pwd;
    if j<10
        filename = ['IMU_00', num2str(j), '_P2'];
    else
        filename = ['IMU_0', num2str(j), '_P2'];
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

baseIMUName = 'pelvis_imu';Placer_xml = [OutPath, '\myIMUPlacer_Setup.xml'];

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

%% PCA Axis Heading Correction - Offset model------------------------------------------------------------------------------------------------------
if ~LL_only
    run humerus_left_alignment_12.m
    run humerus_right_alignment_12.m
    run radius_left_alignment_12.m
    run radius_right_alignment_12.m
end
clc
run Pelvis_alignment_15.m
run Femur_right_alignment_15.m
run tibia_r_alignment_15.m
run toes_r_alignment_15.m
run Femur_left_alignment_15.m
run tibia_l_alignment_15.m
run toes_l_alignment_15.m
run Torso_alignment_15.m

close all
disp(' ')
disp('------------ PCA completed -----------')

if LL_only
    q_axisRot_hr = 0*q_axisRot_t;
    q_axisRot_rr = 0*q_axisRot_t;
    q_axisRot_hl = 0*q_axisRot_t;
    q_axisRot_rl = 0*q_axisRot_t;
end

q_rotations(1, :, j-n) = compact(q_axisRot_t);
q_rotations(2, :, j-n) = compact(q_axisRot_p);
q_rotations(3, :, j-n) = compact(q_axisRot_fr);
q_rotations(4, :, j-n) = compact(q_axisRot_fl);
q_rotations(5, :, j-n) = compact(q_axisRot_tr);
q_rotations(6, :, j-n) = compact(q_axisRot_tl);
q_rotations(7, :, j-n) = compact(q_axisRot_car);
q_rotations(8, :, j-n) = compact(q_axisRot_cal);
q_rotations(9, :, j-n) = compact(q_axisRot_hr);
q_rotations(10, :, j-n) = compact(q_axisRot_hl);
q_rotations(11, :, j-n) = compact(q_axisRot_rr);
q_rotations(12, :, j-n) = compact(q_axisRot_rl);


all_same = "false";% rotation among Z of different angle based on PCA + sit-to-stand/walking

%init
angleLT = 0;
angleRT = 0;
angleLC = 0;
angleRC = 0;


if LL_only
    run OpenIMUs_Placer_LL_v8.m
else
    run OpenIMUs_Placer_v8.m
end
%% LOWERLIMBS ONLY
if LL_only 
    DSt.humerus_r.Q_GS = 0*DSt.torso.Q_GS;
    DSt.humerus_l.Q_GS = 0*DSt.torso.Q_GS;
    DSt.radius_r.Q_GS = 0*DSt.torso.Q_GS;
    DSt.radius_l.Q_GS = 0*DSt.torso.Q_GS;
end


%% Create Matrix   - Xsens (PCA ONLY)

DSt.Data_Matrix_cal = [compact(DSt.torso.Q_GS(fr_cal)), compact(DSt.pelvis.Q_GS(fr_cal)), ...
    compact(DSt.humerus_r.Q_GS(fr_cal)), compact(DSt.radius_r.Q_GS(fr_cal))...
    compact(DSt.humerus_l.Q_GS(fr_cal)), compact(DSt.radius_l.Q_GS(fr_cal))...
    compact(DSt.femur_r.Q_GS(fr_cal)), compact(DSt.tibia_r.Q_GS(fr_cal)), compact(DSt.calcn_r.Q_GS(fr_cal)),...
    compact(DSt.femur_l.Q_GS(fr_cal)), compact(DSt.tibia_l.Q_GS(fr_cal)), compact(DSt.calcn_l.Q_GS(fr_cal))];

DSt.Data_Matrix = [compact(DSt.torso.Q_GS(fr_mov)), compact(DSt.pelvis.Q_GS(fr_mov)),...
    compact(DSt.humerus_r.Q_GS(fr_mov)), compact(DSt.radius_r.Q_GS(fr_mov))...
    compact(DSt.humerus_l.Q_GS(fr_mov)), compact(DSt.radius_l.Q_GS(fr_mov))...
    compact(DSt.femur_r.Q_GS(fr_mov)), compact(DSt.tibia_r.Q_GS(fr_mov)), compact(DSt.calcn_r.Q_GS(fr_mov)),...
    compact(DSt.femur_l.Q_GS(fr_mov)), compact(DSt.tibia_l.Q_GS(fr_mov)), compact(DSt.calcn_l.Q_GS(fr_mov))];


%% Create .STO file
Create_IMU_Storage(fullfile(OutPath, [filename '_orientations_cal.sto']), 1, nbodies, 0, DSt.Data_Matrix_cal, Headers);
Create_IMU_Storage(fullfile(OutPath, [filename '_orientations_trial.sto']), nfr, nbodies, DSt.time, DSt.Data_Matrix, Headers);
pause(1)

%% Q calibration
    q_calibration(1,:, j-n) = compact(qrotx*DSt.torso.Q_GS(fr_cal));
    q_calibration(2,:, j-n) = compact(qrotx*DSt.pelvis.Q_GS(fr_cal));
    q_calibration(3,:, j-n) = compact(qrotx*DSt.femur_r.Q_GS(fr_cal));
    q_calibration(4,:, j-n) = compact(qrotx*DSt.femur_l.Q_GS(fr_cal));
    q_calibration(5,:, j-n) = compact(qrotx*DSt.tibia_r.Q_GS(fr_cal));
    q_calibration(6,:, j-n) = compact(qrotx*DSt.tibia_l.Q_GS(fr_cal));
    q_calibration(7,:, j-n) = compact(qrotx*DSt.calcn_r.Q_GS(fr_cal));
    q_calibration(8,:, j-n) = compact(qrotx*DSt.calcn_l.Q_GS(fr_cal));
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
align_anglesM = nan;
%% Calibrate Model
coord_estimation = false;
visualizeIMU = false;
if s<10
    modelFileName = [ OutPath, '\S0', num2str(s),'_allMarkers_allIMU_CoordEst.osim'];
else
    modelFileName = [ OutPath, '\S', num2str(s),'_allMarkers_allIMU_CoordEst.osim'];
end
% if s<10
%     modelFileName = [ OutPath, '\S0', num2str(s),'_allMarkers_allIMU.osim'];
% else
%     modelFileName = [ OutPath, '\S', num2str(s),'_allMarkers_allIMU.osim'];
% end
run OpenIMUs_CE_v1.m
