# OpenIMUs

This workflow improves the IMU Placer calibration of OpenSense by adding dynamic movements to the calibration.

This work is related to the following paper:

Inertial Sensor-to-Segment Calibration for Accurate 3D Joint Angle Calculation for Use in OpenSim
https://www.mdpi.com/1424-8220/22/9/3259


Funcional calibration method for using from OpenSim 4.1 and MATLAB 2020a on

Steps for the calibration:

The data were collected with Xsens Awinda with Xsens MVN Analyzer Pro 2020.0 (MVN files) and/or MT Manager 2019.2 (mtb files)

1) Convert MVN to MVNX file - MVN software: https://tutorial.xsens.com/video/import-motion-data-into-opensim/
2) Create STO/MAT file based on the number of segments/sensors you want to analyse <- CreateOpenSense_fun_1.mat
  For 1) and 2) follow the example on the MVNX_to_STO repository
  
3) Create accelerometer/gyroscope/magnetometer/quaternions tables <- import_sto_MVN.mat

Steps 1) 2) and 3) are preprocessing steps to have the sensors data in mat format and tables. You can also create your own pipeline.

4) Create DATA Structure for the funcitonal calibration - see Data_Structure_description.xlsx sheet <- Data_Struct_Save_Load_S01.mat
	  -> you need to define manually the functional movement intervals:
e.g.: 
standing_range = 1:100
sit_range = 100:1500
adduction_r_range = 1740:1920

see the Data_Structure folder for the example.


5) Calibration <-OpenIMUs_FC_Mx:
x=1: uses STS + abduction-adduction; 
x=2: uses STS + walking;
x=3: uses STS + abduction-adduction + walking;
    
You can choose which sensors you want to calibrate in you OpenSim model

INPUT: OpenSim model with IMU attached to the relative body segments (any orietation/translation -> they will be corrected) - https://simtk-confluence.stanford.edu:8443/display/OpenSim/How+to+Use+the+IMU+Placer

The workflow was tested on Hamner2010 model modified with 3dof in the knee -> knee_adduction and knee_rotation

OUTPUT: model with IMU correctly oriented (calibrated)

6) OpenIMUs_IK_Mx:
x=1: uses STS + abduction-adduction; 
x=2: uses STS + walking;
x=3: uses STS + abduction-adduction + walking;

This tool use the standatd IMU_IK OpenSense https://simtk-confluence.stanford.edu:8443/display/OpenSim/IMU+Inverse+Kinematics

INPUT: .sto file + calibrated model
OUTPUT: ik.mot file

For any further information contact:
giacomo.diraimondo@kuleuven.be
