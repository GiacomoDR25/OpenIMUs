%%  CREATE IK .MOT
%% IMU Inverse Kinematics - CALIBRATED FILES (WALKING)
clc
for j = 4:6
% IMU registration
close all;
OutPath = pwd;
if j<10
    filename = ['IMU_00', num2str(j), '_P2'];
else
    filename = ['IMU_0', num2str(j), '_P2'];
end
if s<10
    modelFileName_IK = [ OutPath,'\S0', num2str(s),'_allMarkers_allIMU_CoordEst_calibrated.osim'];
else
    modelFileName_IK = [ OutPath,'\S', num2str(s),'_allMarkers_allIMU_CoordEst_calibrated.osim'];
end

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
tic
OutPath = pwd;
if s<10
    modelFileName_IK = [ OutPath,'\S0', num2str(s),'_allMarkers_allIMU_CoordEst_calibrated.osim'];
else
    modelFileName_IK = [ OutPath,'\S', num2str(s),'_allMarkers_allIMU_CoordEst_calibrated.osim'];
end
for j = 1:23
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
% tic
% for j = 12:21
%     
%     DSt = getfield(DS, ['d', num2str(j)]);
% 
% % IMU registration
% close all;
% OutPath = pwd;
%     if j<10
%         filename = ['IMU_00', num2str(j), '_P2'];
%     else
%         filename = ['IMU_0', num2str(j), '_P2'];
%     end
% fr = fs;
% % Headers = {'torso_imu','pelvis_imu', 'femur_r_imu','tibia_r_imu','toes_r_imu','femur_l_imu','tibia_l_imu','toes_l_imu'};
% Headers = {'torso_imu','pelvis_imu','humerus_r_imu','radius_r_imu','humerus_l_imu','radius_l_imu', 'femur_r_imu','tibia_r_imu','calcn_r_imu','femur_l_imu','tibia_l_imu','calcn_l_imu'};
% nbodies =  length(Headers);
% fr_cal = 10;
% fr_mov = 1:length(DSt.time);
% nfr = size(DSt.time, 1);
% 
% baseIMUName = 'pelvis_imu';Placer_xml = [OutPath, '\myIMUPlacer_Setup.xml'];
% % modelFileName = [ OutPath, '\S05_allMarkers_allIMU.osim'];
% modelFileName = [ OutPath, '\S04_allMarkers_allIMU_CoordEst.osim'];
% % modelFileName_IK = [ OutPath,'\S01_allMarkers_allIMU_calibrated.osim'];
% modelFileName_IK = [ OutPath,'\S04_allMarkers_allIMU_CoordEst_calib_Py.osim'];
% visualizeTracking = false;
% calibrate = false;
% % calibrate = true;
% 
% %% PCA Axis Heading Correction - Offset model------------------------------------------------------------------------------------------------------
% run humerus_left_alignment_12.m
% run humerus_right_alignment_12.m
% run radius_left_alignment_12.m
% run radius_right_alignment_12.m
% run Pelvis_alignment_15_notrunk_rot.m
% run Femur_right_alignment_15.m
% run tibia_r_alignment_15.m
% run toes_r_alignment_15.m
% run Femur_left_alignment_15.m
% run tibia_l_alignment_15.m
% run toes_l_alignment_15.m
% run Torso_alignment_15_notrunk_rot.m
% 
% close all
% disp(' ')
% disp('------------ PCA completed -----------')
% 
% 
% % PCA Xsens-----------------------------------------------------------------------------------------------------------------------------------------------------------
% all_same = "false";% rotation among Z of different angle based on PCA + sit-to-stand/walking
% 
% %init
% angleLT = 0;
% angleRT = 0;
% angleLC = 0;
% angleRC = 0;
% 
% % run IMU_Placer_2_IMUv3
% run OpenIMUs_Placer_v8.m
% % 
% % % Orientation in the model with PCA
% % run IMU_Placer_2_v7% calculate global orientation GS without Favre
% 
% %% Create Matrix   - Xsens (PCA ONLY)
% 
% DSt.Data_Matrix_cal = [compact(DSt.torso.Q_GS(fr_cal)), compact(DSt.pelvis.Q_GS(fr_cal)), ...
%     compact(DSt.humerus_r.Q_GS(fr_cal)), compact(DSt.radius_r.Q_GS(fr_cal))...
%     compact(DSt.humerus_l.Q_GS(fr_cal)), compact(DSt.radius_l.Q_GS(fr_cal))...
%     compact(DSt.femur_r.Q_GS(fr_cal)), compact(DSt.tibia_r.Q_GS(fr_cal)), compact(DSt.calcn_r.Q_GS(fr_cal)),...
%     compact(DSt.femur_l.Q_GS(fr_cal)), compact(DSt.tibia_l.Q_GS(fr_cal)), compact(DSt.calcn_l.Q_GS(fr_cal))];
% 
% DSt.Data_Matrix = [compact(DSt.torso.Q_GS(fr_mov)), compact(DSt.pelvis.Q_GS(fr_mov)),...
%     compact(DSt.humerus_r.Q_GS(fr_mov)), compact(DSt.radius_r.Q_GS(fr_mov))...
%     compact(DSt.humerus_l.Q_GS(fr_mov)), compact(DSt.radius_l.Q_GS(fr_mov))...
%     compact(DSt.femur_r.Q_GS(fr_mov)), compact(DSt.tibia_r.Q_GS(fr_mov)), compact(DSt.calcn_r.Q_GS(fr_mov)),...
%     compact(DSt.femur_l.Q_GS(fr_mov)), compact(DSt.tibia_l.Q_GS(fr_mov)), compact(DSt.calcn_l.Q_GS(fr_mov))];
% 
% 
% %% Create .STO file
% Create_IMU_Storage(fullfile(OutPath, [filename '_orientations_cal.sto']), 1, nbodies, 0, DSt.Data_Matrix_cal, Headers);
% Create_IMU_Storage(fullfile(OutPath, [filename '_orientations_trial.sto']), nfr, nbodies, DSt.time, DSt.Data_Matrix, Headers);
% pause(1)
% 
% 
% %% IMU Inverse Kinematics
% clc;
% % IMU registration
% close all;
% orientationsFileName_IK = [filename '_orientations_trial.sto'];
% startTime = 0;
% endTime = 100;
% resultsDirectory = 'IKResults';
% IK_xml = 'myIMUIK_Setup.xml';
% OpenSense_folder = OutPath;
% 
% OpenSense_OrientationTracking_fx(modelFileName_IK, orientationsFileName_IK, ...
%     startTime, endTime, resultsDirectory, visualizeTracking, IK_xml, OpenSense_folder)
% end
% toc
% beep;
% 
