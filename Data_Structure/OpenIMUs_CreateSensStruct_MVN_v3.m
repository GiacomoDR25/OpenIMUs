function [sensors_data] = OpenIMUs_CreateSensStruct_MVN_v3(fs, a, m, q)

sensors_data = struct;
sensors_data.fs = fs;
nF = size(a.time,1);
sensors_data.time = 1/fs * [0:nF-1]';

IMU = imuSensor('accel-gyro-mag', 'SampleRate', fs, 'ReferenceFrame', 'NED');

%% XSENS sensor fusion
% torso
tor.a_free = [a.torso_imuX , a.torso_imuY, a.torso_imuZ];% raw acc
tor.m = [m.torso_imuX , m.torso_imuY, m.torso_imuZ];% raw gyro
tor.q = [q.torso_imu_q0 , q.torso_imu_q1, q.torso_imu_q2, q.torso_imu_q3];% quat Xsens fusion
tor.Q = quaternion(tor.q);% convert format
tor.R = quat2rotm(tor.q);% rotation matrix

tor.g = angvel(tor.Q, 1/fs, 'point');% gyroscope
tor.g(1,:) = tor.g(2,:);
tor.g(end,:) = tor.g(end-1,:);

tor.g_GS = angvel(tor.Q, 1/fs, 'frame');% angular velocity
tor.g_GS(1,:) = tor.g_GS(2,:);
tor.g_GS(end,:) = tor.g_GS(end-1,:);

[tor_acc, gyroReadings, magReadings] = IMU(tor.a_free, tor.g_GS, tor.Q);

for i=1:size(tor.a_free,1) 
    tor.a(i,:) = [tor.a_free(i,1), tor.a_free(i,2), tor.a_free(i,3)+9.81]*tor.R(:,:,i);% acceleration not free
end

R = tor.R;
O = repmat([0,0,1.5] , [nF,1]);
T(1:3,1:3,:) = R;
T(1:3,4,:) = O';
T(4,1:4,:) = repmat([0, 0, 0, 1], [1,1,nF]);
tor.T = T;

% pelvis
pel.a_free = [a.pelvis_imuX , a.pelvis_imuY, a.pelvis_imuZ];
pel.m = [m.pelvis_imuX , m.pelvis_imuY, m.pelvis_imuZ];
pel.q = [q.pelvis_imu_q0 , q.pelvis_imu_q1, q.pelvis_imu_q2, q.pelvis_imu_q3];
pel.Q = quaternion(pel.q);
pel.R = quat2rotm(pel.q);

pel.g = angvel(pel.Q, 1/fs, 'point');
pel.g(1,:) = pel.g(2,:);
pel.g(end,:) = pel.g(end-1,:);

pel.g_GS = angvel(pel.Q, 1/fs, 'frame');
pel.g_GS(1,:) = pel.g_GS(2,:);
pel.g_GS(end,:) = pel.g_GS(end-1,:);

[pel_acc, gyroReadings, magReadings] = IMU(pel.a_free, pel.g_GS, pel.Q);

for i=1:size(pel.a_free,1) 
    pel.a(i,:) = [pel.a_free(i,1), pel.a_free(i,2), pel.a_free(i,3)+9.81]*pel.R(:,:,i);
end

R = pel.R;
O = repmat([0,0,1] , [nF,1]);
T(1:3,1:3,:) = R;
T(1:3,4,:) = O';
T(4,1:4,:) = repmat([0, 0, 0, 1], [1,1,nF]);
pel.T = T;

%femur L
feml.a_free = [a.femur_l_imuX, a.femur_l_imuY, a.femur_l_imuZ, ];
feml.m = [m.femur_l_imuX, m.femur_l_imuY, m.femur_l_imuZ, ];
feml.q = [q.femur_l_imu_q0 , q.femur_l_imu_q1, q.femur_l_imu_q2, q.femur_l_imu_q3];
feml.Q = quaternion(feml.q);
feml.R = quat2rotm(feml.q);

feml.g = angvel(feml.Q, 1/fs, 'point');
feml.g(1,:) = feml.g(2,:);
feml.g(end,:) = feml.g(end-1,:);

feml.g_GS = angvel(feml.Q, 1/fs, 'frame');
feml.g_GS(1,:) = feml.g_GS(2,:);
feml.g_GS(end,:) = feml.g_GS(end-1,:);

[feml_acc, gyroReadings, magReadings] = IMU(feml.a_free, feml.g_GS, feml.Q);

for i=1:size(feml.a_free,1) 
    feml.a(i,:) = [feml.a_free(i,1), feml.a_free(i,2), feml.a_free(i,3)+9.81]*feml.R(:,:,i);
end

R = feml.R;
O = repmat([0, 0.2, 0.85] , [nF,1]);
T(1:3,1:3,:) = R;
T(1:3,4,:) = O';
T(4,1:4,:) = repmat([0, 0, 0, 1], [1,1,nF]);
feml.T = T;

% tibia L
tibl.a_free = [a.tibia_l_imuX, a.tibia_l_imuY, a.tibia_l_imuZ]; 
tibl.m = [m.tibia_l_imuX, m.tibia_l_imuY, m.tibia_l_imuZ]; 
tibl.q = [q.tibia_l_imu_q0 , q.tibia_l_imu_q1, q.tibia_l_imu_q2, q.tibia_l_imu_q3];
tibl.Q = quaternion(tibl.q);
tibl.R = quat2rotm(tibl.q);

tibl.g = angvel(tibl.Q, 1/fs, 'point');
tibl.g(1,:) = tibl.g(2,:);
tibl.g(end,:) = tibl.g(end-1,:);

tibl.g_GS = angvel(tibl.Q, 1/fs, 'frame');
tibl.g_GS(1,:) = tibl.g_GS(2,:);
tibl.g_GS(end,:) = tibl.g_GS(end-1,:);

[tibl_acc, gyroReadings, magReadings] = IMU(tibl.a_free, tibl.g_GS, tibl.Q);

for i=1:size(tibl.a_free,1) 
    tibl.a(i,:) = [tibl.a_free(i,1), tibl.a_free(i,2), tibl.a_free(i,3)+9.81]*tibl.R(:,:,i);
end

R = tibl.R;
O = repmat([0,0.2,0.4] , [nF,1]);
T(1:3,1:3,:) = R;
T(1:3,4,:) = O';
T(4,1:4,:) = repmat([0, 0, 0, 1], [1,1,nF]);
tibl.T = T;

% % toes L
% toel.a_free = [a.toes_l_imuX, a.toes_l_imuY, a.toes_l_imuZ]; 
% toel.m = [m.toes_l_imuX, m.toes_l_imuY, m.toes_l_imuZ]; 
% toel.q = [q.toes_l_imu_q0 , q.toes_l_imu_q1, q.toes_l_imu_q2, q.toes_l_imu_q3];
% toel.Q = quaternion(toel.q);
% toel.R = quat2rotm(toel.q);
% 
% toel.g = angvel(toel.Q, 1/fs, 'point');
% toel.g(1,:) = toel.g(2,:);
% toel.g(end,:) = toel.g(end-1,:);
% 
% toel.g_GS = angvel(toel.Q, 1/fs, 'frame');
% toel.g_GS(1,:) = toel.g_GS(2,:);
% toel.g_GS(end,:) = toel.g_GS(end-1,:);
% 
% [toel_acc, gyroReadings, magReadings] = IMU(toel.a_free, toel.g_GS, toel.Q);
% 
% for i=1:size(toel.a_free,1) 
%     toel.a(i,:) = [toel.a_free(i,1), toel.a_free(i,2), toel.a_free(i,3)+9.81]*toel.R(:,:,i);
% end
% 
% R = toel.R;
% O = repmat([0,0.2,0.1] , [nF,1]);
% T(1:3,1:3,:) = R;
% T(1:3,4,:) = O';
% T(4,1:4,:) = repmat([0, 0, 0, 1], [1,1,nF]);
% toel.T = T;

% calcn L
call.a_free = [a.calcn_l_imuX, a.calcn_l_imuY, a.calcn_l_imuZ]; 
call.m = [m.calcn_l_imuX, m.calcn_l_imuY, m.calcn_l_imuZ]; 
call.q = [q.calcn_l_imu_q0 , q.calcn_l_imu_q1, q.calcn_l_imu_q2, q.calcn_l_imu_q3];
call.Q = quaternion(call.q);
call.R = quat2rotm(call.q);

call.g = angvel(call.Q, 1/fs, 'point');
call.g(1,:) = call.g(2,:);
call.g(end,:) = call.g(end-1,:);

call.g_GS = angvel(call.Q, 1/fs, 'frame');
call.g_GS(1,:) = call.g_GS(2,:);
call.g_GS(end,:) = call.g_GS(end-1,:);

[call_acc, gyroReadings, magReadings] = IMU(call.a_free, call.g_GS, call.Q);

for i=1:size(call.a_free,1) 
    call.a(i,:) = [call.a_free(i,1), call.a_free(i,2), call.a_free(i,3)+9.81]*call.R(:,:,i);
end

R = call.R;
O = repmat([0,0.2,0.1] , [nF,1]);
T(1:3,1:3,:) = R;
T(1:3,4,:) = O';
T(4,1:4,:) = repmat([0, 0, 0, 1], [1,1,nF]);
call.T = T;

%femur R
femr.a_free = [a.femur_r_imuX, a.femur_r_imuY, a.femur_r_imuZ];
femr.m = [m.femur_r_imuX, m.femur_r_imuY, m.femur_r_imuZ];
femr.q = [q.femur_r_imu_q0 , q.femur_r_imu_q1, q.femur_r_imu_q2, q.femur_r_imu_q3];
femr.Q = quaternion(femr.q);
femr.R = quat2rotm(femr.q);

femr.g = angvel(femr.Q, 1/fs, 'point');
femr.g(1,:) = femr.g(2,:);
femr.g(end,:) = femr.g(end-1,:);

femr.g_GS = angvel(femr.Q, 1/fs, 'frame');
femr.g_GS(1,:) = femr.g_GS(2,:);
femr.g_GS(end,:) = femr.g_GS(end-1,:);

[femr_acc, gyroReadings, magReadings] = IMU(femr.a_free, femr.g_GS, femr.Q);

for i=1:size(femr.a_free,1) 
    femr.a(i,:) = [femr.a_free(i,1), femr.a_free(i,2), femr.a_free(i,3)+9.81]*femr.R(:,:,i);
end


R = femr.R;
O = repmat([0,-0.2,0.85] , [nF,1]);
T(1:3,1:3,:) = R;
T(1:3,4,:) = O';
T(4,1:4,:) = repmat([0, 0, 0, 1], [1,1,nF]);
femr.T = T;

% tibia R
tibr.a_free = [a.tibia_r_imuX, a.tibia_r_imuY, a.tibia_r_imuZ]; 
tibr.m = [m.tibia_r_imuX, m.tibia_r_imuY, m.tibia_r_imuZ]; 
tibr.q = [q.tibia_r_imu_q0 , q.tibia_r_imu_q1, q.tibia_r_imu_q2, q.tibia_r_imu_q3];
tibr.Q = quaternion(tibr.q);
tibr.R = quat2rotm(tibr.q);

tibr.g = angvel(tibr.Q, 1/fs, 'point');
tibr.g(1,:) = tibr.g(2,:);
tibr.g(end,:) = tibr.g(end-1,:);

tibr.g_GS = angvel(tibr.Q, 1/fs, 'frame');
tibr.g_GS(1,:) = tibr.g_GS(2,:);
tibr.g_GS(end,:) = tibr.g_GS(end-1,:);

[tibr_acc, gyroReadings, magReadings] = IMU(tibr.a_free, tibr.g_GS, tibr.Q);

for i=1:size(tibr.a_free,1) 
    tibr.a(i,:) = [tibr.a_free(i,1), tibr.a_free(i,2), tibr.a_free(i,3)+9.81]*tibr.R(:,:,i);
end

R = tibr.R;
O = repmat([0,-0.2,0.4] , [nF,1]);
T(1:3,1:3,:) = R;
T(1:3,4,:) = O';
T(4,1:4,:) = repmat([0, 0, 0, 1], [1,1,nF]);
tibr.T = T;

% calcn R
calr.a_free = [a.calcn_r_imuX, a.calcn_r_imuY, a.calcn_r_imuZ]; 
calr.m = [m.calcn_r_imuX, m.calcn_r_imuY, m.calcn_r_imuZ]; 
calr.q = [q.calcn_r_imu_q0 , q.calcn_r_imu_q1, q.calcn_r_imu_q2, q.calcn_r_imu_q3];
calr.Q = quaternion(calr.q);
calr.R = quat2rotm(calr.q);

calr.g = angvel(calr.Q, 1/fs, 'point');
calr.g(1,:) = calr.g(2,:);
calr.g(end,:) = calr.g(end-1,:);

calr.g_GS = angvel(calr.Q, 1/fs, 'frame');
calr.g_GS(1,:) = calr.g_GS(2,:);
calr.g_GS(end,:) = calr.g_GS(end-1,:);

[calr_acc, gyroReadings, magReadings] = IMU(calr.a_free, calr.g_GS, calr.Q);

for i=1:size(calr.a_free,1) 
    calr.a(i,:) = [calr.a_free(i,1), calr.a_free(i,2), calr.a_free(i,3)+9.81]*calr.R(:,:,i);
end

R = calr.R;
O = repmat([0,-0.2,0.1] , [nF,1]);
T(1:3,1:3,:) = R;
T(1:3,4,:) = O';
T(4,1:4,:) = repmat([0, 0, 0, 1], [1,1,nF]);
calr.T = T;

% % toes R
% toer.a_free = [a.toes_r_imuX, a.toes_r_imuY, a.toes_r_imuZ]; 
% toer.m = [m.toes_r_imuX, m.toes_r_imuY, m.toes_r_imuZ]; 
% toer.q = [q.toes_r_imu_q0 , q.toes_r_imu_q1, q.toes_r_imu_q2, q.toes_r_imu_q3];
% toer.Q = quaternion(toer.q);
% toer.R = quat2rotm(toer.q);
% 
% toer.g = angvel(toer.Q, 1/fs, 'point');
% toer.g(1,:) = toer.g(2,:);
% toer.g(end,:) = toer.g(end-1,:);
% 
% toer.g_GS = angvel(toer.Q, 1/fs, 'frame');
% toer.g_GS(1,:) = toer.g_GS(2,:);
% toer.g_GS(end,:) = toer.g_GS(end-1,:);
% 
% [toer_acc, gyroReadings, magReadings] = IMU(toer.a_free, toer.g_GS, toer.Q);
% 
% for i=1:size(toer.a_free,1) 
%     toer.a(i,:) = [toer.a_free(i,1), toer.a_free(i,2), toer.a_free(i,3)+9.81]*toer.R(:,:,i);
% end
% 
% R = toer.R;
% O = repmat([0,-0.2,0.1] , [nF,1]);
% T(1:3,1:3,:) = R;
% T(1:3,4,:) = O';
% T(4,1:4,:) = repmat([0, 0, 0, 1], [1,1,nF]);
% toer.T = T;
% 

%humerus L
huml.a_free = [a.humerus_l_imuX, a.humerus_l_imuY, a.humerus_l_imuZ, ];
huml.m = [m.humerus_l_imuX, m.humerus_l_imuY, m.humerus_l_imuZ, ];
huml.q = [q.humerus_l_imu_q0 , q.humerus_l_imu_q1, q.humerus_l_imu_q2, q.humerus_l_imu_q3];
huml.Q = quaternion(huml.q);
huml.R = quat2rotm(huml.q);

huml.g = angvel(huml.Q, 1/fs, 'point');
huml.g(1,:) = huml.g(2,:);
huml.g(end,:) = huml.g(end-1,:);

huml.g_GS = angvel(huml.Q, 1/fs, 'frame');
huml.g_GS(1,:) = huml.g_GS(2,:);
huml.g_GS(end,:) = huml.g_GS(end-1,:);

[huml_acc, gyroReadings, magReadings] = IMU(huml.a_free, huml.g_GS, huml.Q);

for i=1:size(huml.a_free,1) 
    huml.a(i,:) = [huml.a_free(i,1), huml.a_free(i,2), huml.a_free(i,3)+9.81]*huml.R(:,:,i);
end

R = huml.R;
O = repmat([0, 0.3, 1.7] , [nF,1]);
T(1:3,1:3,:) = R;
T(1:3,4,:) = O';
T(4,1:4,:) = repmat([0, 0, 0, 1], [1,1,nF]);
huml.T = T;

%humerus R
humr.a_free = [a.humerus_r_imuX, a.humerus_r_imuY, a.humerus_r_imuZ, ];
humr.m = [m.humerus_r_imuX, m.humerus_r_imuY, m.humerus_r_imuZ, ];
humr.q = [q.humerus_r_imu_q0 , q.humerus_r_imu_q1, q.humerus_r_imu_q2, q.humerus_r_imu_q3];
humr.Q = quaternion(humr.q);
humr.R = quat2rotm(humr.q);

humr.g = angvel(humr.Q, 1/fs, 'point');
humr.g(1,:) = humr.g(2,:);
humr.g(end,:) = humr.g(end-1,:);

humr.g_GS = angvel(humr.Q, 1/fs, 'frame');
humr.g_GS(1,:) = humr.g_GS(2,:);
humr.g_GS(end,:) = humr.g_GS(end-1,:);

[humr_acc, gyroReadings, magReadings] = IMU(humr.a_free, humr.g_GS, humr.Q);

for i=1:size(humr.a_free,1) 
    humr.a(i,:) = [humr.a_free(i,1), humr.a_free(i,2), humr.a_free(i,3)+9.81]*humr.R(:,:,i);
end

R = humr.R;
O = repmat([0, -0.3, 1.7] , [nF,1]);
T(1:3,1:3,:) = R;
T(1:3,4,:) = O';
T(4,1:4,:) = repmat([0, 0, 0, 1], [1,1,nF]);
humr.T = T;

%radius L
radl.a_free = [a.radius_l_imuX, a.radius_l_imuY, a.radius_l_imuZ, ];
radl.m = [m.radius_l_imuX, m.radius_l_imuY, m.radius_l_imuZ, ];
radl.q = [q.radius_l_imu_q0 , q.radius_l_imu_q1, q.radius_l_imu_q2, q.radius_l_imu_q3];
radl.Q = quaternion(radl.q);
radl.R = quat2rotm(radl.q);

radl.g = angvel(radl.Q, 1/fs, 'point');
radl.g(1,:) = radl.g(2,:);
radl.g(end,:) = radl.g(end-1,:);

radl.g_GS = angvel(radl.Q, 1/fs, 'frame');
radl.g_GS(1,:) = radl.g_GS(2,:);
radl.g_GS(end,:) = radl.g_GS(end-1,:);

[radl_acc, gyroReadings, magReadings] = IMU(radl.a_free, radl.g_GS, radl.Q);

for i=1:size(radl.a_free,1) 
    radl.a(i,:) = [radl.a_free(i,1), radl.a_free(i,2), radl.a_free(i,3)+9.81]*radl.R(:,:,i);
end

R = radl.R;
O = repmat([0, 0.3, 1.2] , [nF,1]);
T(1:3,1:3,:) = R;
T(1:3,4,:) = O';
T(4,1:4,:) = repmat([0, 0, 0, 1], [1,1,nF]);
radl.T = T;

% %radius R
radr.a_free = [a.radius_r_imuX, a.radius_r_imuY, a.radius_r_imuZ, ];
radr.m = [m.radius_r_imuX, m.radius_r_imuY, m.radius_r_imuZ, ];
radr.q = [q.radius_r_imu_q0 , q.radius_r_imu_q1, q.radius_r_imu_q2, q.radius_r_imu_q3];
radr.Q = quaternion(radr.q);
radr.R = quat2rotm(radr.q);

radr.g = angvel(radr.Q, 1/fs, 'point');
radr.g(1,:) = radr.g(2,:);
radr.g(end,:) = radr.g(end-1,:);

radr.g_GS = angvel(radr.Q, 1/fs, 'frame');
radr.g_GS(1,:) = radr.g_GS(2,:);
radr.g_GS(end,:) = radr.g_GS(end-1,:);

[radr_acc, gyroReadings, magReadings] = IMU(radr.a_free, radr.g_GS, radr.Q);

for i=1:size(radr.a_free,1) 
    radr.a(i,:) = [radr.a_free(i,1), radr.a_free(i,2), radr.a_free(i,3)+9.81]*radr.R(:,:,i);
end

R = radr.R;
O = repmat([0, -0.3, 1.2] , [nF,1]);
T(1:3,1:3,:) = R;
T(1:3,4,:) = O';
T(4,1:4,:) = repmat([0, 0, 0, 1], [1,1,nF]);
radr.T = T;



%plot
RF = 1;
figure;
subplot(1,2,1);
hold on; box on; grid on; grid minor;
plotH(0.25, eye(4), [0.95 0.4 0.2]);
plotH(0.15, huml.T(:,:,RF), [0.95 0.2 0.4]);
plotH(0.15, radl.T(:,:,RF), [0.95 0.3 0.5]);
plotH(0.15, humr.T(:,:,RF), [0.2 0.95 0.55]);
plotH(0.15, radr.T(:,:,RF), [0.4 0.85 0.55]);
plotH(0.15, tor.T(:,:,RF), [0.2 0.6 0.95]);
plotH(0.15, pel.T(:,:,RF), [0.4 0.6 0.95]);
plotH(0.15, feml.T(:,:,RF), [0.95 0.2 0.4]);
plotH(0.15, tibl.T(:,:,RF), [0.95 0.3 0.5]);
plotH(0.15, call.T(:,:,RF), [0.95 0.4 0.5]);
plotH(0.15, femr.T(:,:,RF), [0.2 0.95 0.55]);
plotH(0.15, tibr.T(:,:,RF), [0.4 0.85 0.55]);
plotH(0.15, calr.T(:,:,RF), [0.6 0.85 0.55]);
axis equal;xlabel('X_{Xsens}');ylabel('Y_{Xsens}');zlabel('Z_{Xsens}');
title('Orientation of sensors in Xsens GS (Xsens AHRS Kalman filter)');

%% IMU filter sensors fusion

gyr_noise = (0.01*sqrt(fs)*(pi/180))^2; % Xsens specification 0.01deg/s/sqrt(Hz)
acc_noise = ((200e-6)/sqrt(fs))^2; % Xsens specification 200e-6g/sqrt(Hz)
drift_bias = (10/3600*(pi/180))^2;% Xsens specification 10deg/hr

% Build the Kalman filter (acc + gyr)
ifilt = imufilter('SampleRate', fs, 'ReferenceFrame', 'NED',...
    'AccelerometerNoise', acc_noise,...
    'GyroscopeNoise', gyr_noise,...
    'GyroscopeDriftNoise', drift_bias,...
    'DecimationFactor', 1);
humr.fQ = ifilt(humr.a, humr.g);% humerus right
humr.fq = compact(humr.fQ);
humr.g_fGS = angvel(humr.fQ, 1/fs, 'frame');
humr.g_fGS(1,:) = humr.g_fGS(2,:);
humr.g_fGS(end,:) = humr.g_fGS(end-1,:);

ifilt = imufilter('SampleRate', fs, 'ReferenceFrame', 'NED',...
    'AccelerometerNoise', acc_noise,...
    'GyroscopeNoise', gyr_noise,...
    'GyroscopeDriftNoise', drift_bias,...
    'DecimationFactor', 1);
huml.fQ = ifilt(huml.a, huml.g);% humerus left
huml.fq = compact(huml.fQ);
huml.g_fGS = angvel(huml.fQ, 1/fs, 'frame');
huml.g_fGS(1,:) = huml.g_fGS(2,:);
huml.g_fGS(end,:) = huml.g_fGS(end-1,:);

ifilt = imufilter('SampleRate', fs, 'ReferenceFrame', 'NED',...
    'AccelerometerNoise', acc_noise,...
    'GyroscopeNoise', gyr_noise,...
    'GyroscopeDriftNoise', drift_bias,...
    'DecimationFactor', 1);
radr.fQ = ifilt(radr.a, radr.g);% radius right
radr.fq = compact(radr.fQ);
radr.g_fGS = angvel(radr.fQ, 1/fs, 'frame');
radr.g_fGS(1,:) = radr.g_fGS(2,:);
radr.g_fGS(end,:) = radr.g_fGS(end-1,:);

ifilt = imufilter('SampleRate', fs, 'ReferenceFrame', 'NED',...
    'AccelerometerNoise', acc_noise,...
    'GyroscopeNoise', gyr_noise,...
    'GyroscopeDriftNoise', drift_bias,...
    'DecimationFactor', 1);
radl.fQ = ifilt(radl.a, radl.g);% radius left
radl.fq = compact(radl.fQ);
radl.g_fGS = angvel(radl.fQ, 1/fs, 'frame');
radl.g_fGS(1,:) = radl.g_fGS(2,:);
radl.g_fGS(end,:) = radl.g_fGS(end-1,:);

ifilt = imufilter('SampleRate', fs, 'ReferenceFrame', 'NED',...
    'AccelerometerNoise', acc_noise,...
    'GyroscopeNoise', gyr_noise,...
    'GyroscopeDriftNoise', drift_bias,...
    'DecimationFactor', 1);
tor.fQ = ifilt(tor.a, tor.g);% torso
tor.fq = compact(tor.fQ);
tor.g_fGS = angvel(tor.fQ, 1/fs, 'frame');
tor.g_fGS(1,:) = tor.g_fGS(2,:);
tor.g_fGS(end,:) = tor.g_fGS(end-1,:);

ifilt = imufilter('SampleRate', fs, 'ReferenceFrame', 'NED',...
    'AccelerometerNoise', acc_noise,...
    'GyroscopeNoise', gyr_noise,...
    'GyroscopeDriftNoise', drift_bias,...
    'DecimationFactor', 1);
pel.fQ = ifilt(pel.a, pel.g);% pelvis
pel.fq = compact(pel.fQ);
pel.g_fGS = angvel(pel.fQ, 1/fs, 'frame');
pel.g_fGS(1,:) = pel.g_fGS(2,:);
pel.g_fGS(end,:) = pel.g_fGS(end-1,:);

ifilt = imufilter('SampleRate', fs, 'ReferenceFrame', 'NED',...
    'AccelerometerNoise', acc_noise,...
    'GyroscopeNoise', gyr_noise,...
    'GyroscopeDriftNoise', drift_bias,...
    'DecimationFactor', 1);
feml.fQ = ifilt(feml.a, feml.g);% femur L
feml.fq = compact(feml.fQ);
feml.g_fGS = angvel(feml.fQ, 1/fs, 'frame');
feml.g_fGS(1,:) = feml.g_fGS(2,:);
feml.g_fGS(end,:) = feml.g_fGS(end-1,:);

ifilt = imufilter('SampleRate', fs, 'ReferenceFrame', 'NED',...
    'AccelerometerNoise', acc_noise,...
    'GyroscopeNoise', gyr_noise,...
    'GyroscopeDriftNoise', drift_bias,...
    'DecimationFactor', 1);
femr.fQ = ifilt(femr.a, femr.g);% femur R
femr.fq = compact(femr.fQ);
femr.g_fGS = angvel(femr.fQ, 1/fs, 'frame');
femr.g_fGS(1,:) = femr.g_fGS(2,:);
femr.g_fGS(end,:) = femr.g_fGS(end-1,:);

ifilt = imufilter('SampleRate', fs, 'ReferenceFrame', 'NED',...
    'AccelerometerNoise', acc_noise,...
    'GyroscopeNoise', gyr_noise,...
    'GyroscopeDriftNoise', drift_bias,...
    'DecimationFactor', 1);
tibl.fQ = ifilt(tibl.a, tibl.g); % tibia L
tibl.fq = compact(tibl.fQ);
tibl.g_fGS = angvel(tibl.fQ, 1/fs, 'frame');
tibl.g_fGS(1,:) = tibl.g_fGS(2,:);
tibl.g_fGS(end,:) = tibl.g_fGS(end-1,:);

ifilt = imufilter('SampleRate', fs, 'ReferenceFrame', 'NED',...
    'AccelerometerNoise', acc_noise,...
    'GyroscopeNoise', gyr_noise,...
    'GyroscopeDriftNoise', drift_bias,...
    'DecimationFactor', 1);
tibr.fQ = ifilt(tibr.a, tibr.g);% tibia R
tibr.fq = compact(tibr.fQ);
tibr.g_fGS = angvel(tibr.fQ, 1/fs, 'frame');
tibr.g_fGS(1,:) = tibr.g_fGS(2,:);
tibr.g_fGS(end,:) = tibr.g_fGS(end-1,:);

ifilt = imufilter('SampleRate', fs, 'ReferenceFrame', 'NED',...
    'AccelerometerNoise', acc_noise,...
    'GyroscopeNoise', gyr_noise,...
    'GyroscopeDriftNoise', drift_bias,...
    'DecimationFactor', 1);
call.fQ = ifilt(call.a, call.g);% calcn L
call.fq = compact(call.fQ);
call.g_fGS = angvel(call.fQ, 1/fs, 'frame');
call.g_fGS(1,:) = call.g_fGS(2,:);
call.g_fGS(end,:) = call.g_fGS(end-1,:);

ifilt = imufilter('SampleRate', fs, 'ReferenceFrame', 'NED',...
    'AccelerometerNoise', acc_noise,...
    'GyroscopeNoise', gyr_noise,...
    'GyroscopeDriftNoise', drift_bias,...
    'DecimationFactor', 1);
calr.fQ = ifilt(calr.a, calr.g); %calcn R
calr.fq = compact(calr.fQ);
calr.g_fGS = angvel(calr.fQ, 1/fs, 'frame');
calr.g_fGS(1,:) = calr.g_fGS(2,:);
calr.g_fGS(end,:) = calr.g_fGS(end-1,:);

% Plot Orientation imufilter
humr.fR = quat2rotm(humr.fQ);
fR = humr.fR;
fO = repmat([0, -0.3,1.7] , [nF,1]);
fT(1:3,1:3,:) = fR;
fT(1:3,4,:) = fO';
fT(4,1:4,:) = repmat([0, 0, 0, 1], [1,1,nF]);
humr.fT = fT;

huml.fR = quat2rotm(huml.fQ);
fR = huml.fR;
fO = repmat([0, 0.3,1.7] , [nF,1]);
fT(1:3,1:3,:) = fR;
fT(1:3,4,:) = fO';
fT(4,1:4,:) = repmat([0, 0, 0, 1], [1,1,nF]);
huml.fT = fT;

radr.fR = quat2rotm(radr.fQ);
fR = radr.fR;
fO = repmat([0, -0.3,1.2] , [nF,1]);
fT(1:3,1:3,:) = fR;
fT(1:3,4,:) = fO';
fT(4,1:4,:) = repmat([0, 0, 0, 1], [1,1,nF]);
radr.fT = fT;

radl.fR = quat2rotm(radl.fQ);
fR = radl.fR;
fO = repmat([0, 0.3,1.2] , [nF,1]);
fT(1:3,1:3,:) = fR;
fT(1:3,4,:) = fO';
fT(4,1:4,:) = repmat([0, 0, 0, 1], [1,1,nF]);
radl.fT = fT;

tor.fR = quat2rotm(tor.fQ);
fR = tor.fR;
fO = repmat([0,0,1.5] , [nF,1]);
fT(1:3,1:3,:) = fR;
fT(1:3,4,:) = fO';
fT(4,1:4,:) = repmat([0, 0, 0, 1], [1,1,nF]);
tor.fT = fT;

pel.fR = quat2rotm(pel.fQ);
fR = pel.fR;
fO = repmat([0,0,1] , [nF,1]);
fT(1:3,1:3,:) = fR;
fT(1:3,4,:) = fO';
fT(4,1:4,:) = repmat([0, 0, 0, 1], [1,1,nF]);
pel.fT = fT;

feml.fR = quat2rotm(feml.fQ);
fR = feml.fR;
fO = repmat([0, 0.2, 0.85] , [nF,1]);
fT(1:3,1:3,:) = fR;
fT(1:3,4,:) = fO';
fT(4,1:4,:) = repmat([0, 0, 0, 1], [1,1,nF]);
feml.fT = fT;

tibl.fR = quat2rotm(tibl.fQ);
fR = tibl.fR;
fO = repmat([0, 0.2, 0.4] , [nF,1]);
fT(1:3,1:3,:) = fR;
fT(1:3,4,:) = fO';
fT(4,1:4,:) = repmat([0, 0, 0, 1], [1,1,nF]);
tibl.fT = fT;

call.fR = quat2rotm(call.fQ);
fR = call.fR;
fO = repmat([0, 0.2, 0.1] , [nF,1]);
fT(1:3,1:3,:) = fR;
fT(1:3,4,:) = fO';
fT(4,1:4,:) = repmat([0, 0, 0, 1], [1,1,nF]);
call.fT = fT;

femr.fR = quat2rotm(femr.fQ);
fR = femr.fR;
fO = repmat([0, -0.2, 0.85] , [nF,1]);
fT(1:3,1:3,:) = fR;
fT(1:3,4,:) = fO';
fT(4,1:4,:) = repmat([0, 0, 0, 1], [1,1,nF]);
femr.fT = fT;

tibr.fR = quat2rotm(tibr.fQ);
fR = tibr.fR;
fO = repmat([0, -0.2, 0.4] , [nF,1]);
fT(1:3,1:3,:) = fR;
fT(1:3,4,:) = fO';
fT(4,1:4,:) = repmat([0, 0, 0, 1], [1,1,nF]);
tibr.fT = fT;

calr.fR = quat2rotm(calr.fQ);
fR = calr.fR;
fO = repmat([0, -0.2, 0.1] , [nF,1]);
fT(1:3,1:3,:) = fR;
fT(1:3,4,:) = fO';
fT(4,1:4,:) = repmat([0, 0, 0, 1], [1,1,nF]);
calr.fT = fT;

% plot
subplot(1,2,2);
hold on; box on; grid on; grid minor;
plotH(0.25, eye(4), [0.95 0.4 0.2]);
plotH(0.15, huml.fT(:,:,RF), [0.95 0.2 0.4]);
plotH(0.15, radl.fT(:,:,RF), [0.95 0.3 0.5]);
plotH(0.15, humr.fT(:,:,RF), [0.2 0.95 0.55]);
plotH(0.15, radr.fT(:,:,RF), [0.4 0.85 0.55]);
plotH(0.15, tor.fT(:,:,RF), [0.2 0.6 0.95]);
plotH(0.15, pel.fT(:,:,RF), [0.4 0.6 0.95]);
plotH(0.15, feml.fT(:,:,RF), [0.95 0.2 0.4]);
plotH(0.15, tibl.fT(:,:,RF), [0.95 0.3 0.5]);
plotH(0.15, call.fT(:,:,RF), [0.95 0.4 0.5]);
plotH(0.15, femr.fT(:,:,RF), [0.2 0.95 0.55]);
plotH(0.15, tibr.fT(:,:,RF), [0.4 0.85 0.55]);
plotH(0.15, calr.fT(:,:,RF), [0.6 0.85 0.55]);
axis equal;xlabel('X_{imufilter}');ylabel('Y_{imufilter}');zlabel('Z_{imufilter}');
title('Orientation of sensors in GS (IMUfilter Kalman filter)');

%% Madgwick filter sensors fusion

%torso
% Build Madgwick filter 
Mad_Filt = MadgwickAHRS('SamplePeriod', 1/fs, 'Quaternion', compact(tor.Q(1)) , 'Beta', 0.005);
% AHRS = MahonyAHRS('SamplePeriod', 1/256, 'Kp', 0.5);
tor.qMad = nan(length(sensors_data.time), 4);
for t = 1:length(sensors_data.time)
    Mad_Filt.Update(tor.g(t,:), tor.a(t,:), tor.m(t,:));	% gyroscope units must be radians
    tor.qMad(t, :) = Mad_Filt.Quaternion;
end
tor.QMad = quaternion(tor.qMad);

%pelvis
% Build Madgwick filter 
Mad_Filt = MadgwickAHRS('SamplePeriod', 1/fs, 'Quaternion', compact(pel.Q(1)) , 'Beta', 0.005);
% AHRS = MahonyAHRS('SamplePeriod', 1/256, 'Kp', 0.5);
pel.qMad = nan(length(sensors_data.time), 4);
for t = 1:length(sensors_data.time)
    Mad_Filt.Update(pel.g(t,:), pel.a(t,:), pel.m(t,:));	% gyroscope units must be radians
    pel.qMad(t, :) = Mad_Filt.Quaternion;
end
pel.QMad = quaternion(pel.qMad);

%femu_l
% Build Madgwick filter 
Mad_Filt = MadgwickAHRS('SamplePeriod', 1/fs, 'Quaternion', compact(feml.Q(1)) , 'Beta', 0.005);
% AHRS = MahonyAHRS('SamplePeriod', 1/256, 'Kp', 0.5);
feml.qMad = nan(length(sensors_data.time), 4);
for t = 1:length(sensors_data.time)
    Mad_Filt.Update(feml.g(t,:), feml.a(t,:), feml.m(t,:));	% gyroscope units must be radians
    feml.qMad(t, :) = Mad_Filt.Quaternion;
end
feml.QMad = quaternion(feml.qMad);

%femur_r
% Build Madgwick filter 
Mad_Filt = MadgwickAHRS('SamplePeriod', 1/fs, 'Quaternion', compact(femr.Q(1)) , 'Beta', 0.005);
% AHRS = MahonyAHRS('SamplePeriod', 1/256, 'Kp', 0.5);
femr.qMad = nan(length(sensors_data.time), 4);
for t = 1:length(sensors_data.time)
    Mad_Filt.Update(femr.g(t,:), femr.a(t,:), femr.m(t,:));	% gyroscope units must be radians
    femr.qMad(t, :) = Mad_Filt.Quaternion;
end
femr.QMad = quaternion(femr.qMad);

%tibia_l
% Build Madgwick filter 
Mad_Filt = MadgwickAHRS('SamplePeriod', 1/fs, 'Quaternion', compact(tibl.Q(1)) , 'Beta', 0.005);
% AHRS = MahonyAHRS('SamplePeriod', 1/256, 'Kp', 0.5);
tibl.qMad = nan(length(sensors_data.time), 4);
for t = 1:length(sensors_data.time)
    Mad_Filt.Update(tibl.g(t,:), tibl.a(t,:), tibl.m(t,:));	% gyroscope units must be radians
    tibl.qMad(t, :) = Mad_Filt.Quaternion;
end
tibl.QMad = quaternion(tibl.qMad);

%tibia_r
% Build Madgwick filter 
Mad_Filt = MadgwickAHRS('SamplePeriod', 1/fs, 'Quaternion', compact(tibr.Q(1)) , 'Beta', 0.005);
% AHRS = MahonyAHRS('SamplePeriod', 1/256, 'Kp', 0.5);
tibr.qMad = nan(length(sensors_data.time), 4);
for t = 1:length(sensors_data.time)
    Mad_Filt.Update(tibr.g(t,:), tibr.a(t,:), tibr.m(t,:));	% gyroscope units must be radians
    tibr.qMad(t, :) = Mad_Filt.Quaternion;
end
tibr.QMad = quaternion(tibr.qMad);

%toes_l
% Build Madgwick filter 
Mad_Filt = MadgwickAHRS('SamplePeriod', 1/fs, 'Quaternion', compact(call.Q(1)) , 'Beta', 0.005);
% AHRS = MahonyAHRS('SamplePeriod', 1/256, 'Kp', 0.5);
call.qMad = nan(length(sensors_data.time), 4);
for t = 1:length(sensors_data.time)
    Mad_Filt.Update(call.g(t,:), call.a(t,:), call.m(t,:));	% gyroscope units must be radians
    call.qMad(t, :) = Mad_Filt.Quaternion;
end
call.QMad = quaternion(call.qMad);

%toes_r
% Build Madgwick filter 
Mad_Filt = MadgwickAHRS('SamplePeriod', 1/fs, 'Quaternion', compact(calr.Q(1)) , 'Beta', 0.005);
% AHRS = MahonyAHRS('SamplePeriod', 1/256, 'Kp', 0.5);
calr.qMad = nan(length(sensors_data.time), 4);
for t = 1:length(sensors_data.time)
    Mad_Filt.Update(calr.g(t,:), calr.a(t,:), calr.m(t,:));	% gyroscope units must be radians
    calr.qMad(t, :) = Mad_Filt.Quaternion;
end
calr.QMad = quaternion(calr.qMad);

% % Plot 
% tor.RMad = quat2rotm(tor.fQ);
% fR = tor.fR;
% fO = repmat([0,0,1.5] , [nF,1]);
% fT(1:3,1:3,:) = fR;
% fT(1:3,4,:) = fO';
% fT(4,1:4,:) = repmat([0, 0, 0, 1], [1,1,nF]);
% tor.fT = fT;
% 
% pel.fR = quat2rotm(pel.fQ);
% fR = pel.fR;
% fO = repmat([0,0,1] , [nF,1]);
% fT(1:3,1:3,:) = fR;
% fT(1:3,4,:) = fO';
% fT(4,1:4,:) = repmat([0, 0, 0, 1], [1,1,nF]);
% pel.fT = fT;
% 
% feml.fR = quat2rotm(feml.fQ);
% fR = feml.fR;
% fO = repmat([0, 0.2, 0.85] , [nF,1]);
% fT(1:3,1:3,:) = fR;
% fT(1:3,4,:) = fO';
% fT(4,1:4,:) = repmat([0, 0, 0, 1], [1,1,nF]);
% feml.fT = fT;
% 
% femr.fR = quat2rotm(femr.fQ);
% fR = femr.fR;
% fO = repmat([0, -0.2, 0.85] , [nF,1]);
% fT(1:3,1:3,:) = fR;
% fT(1:3,4,:) = fO';
% fT(4,1:4,:) = repmat([0, 0, 0, 1], [1,1,nF]);
% femr.fT = fT;
% 
% tibl.fR = quat2rotm(tibl.fQ);
% fR = tibl.fR;
% fO = repmat([0, 0.2, 0.4] , [nF,1]);
% fT(1:3,1:3,:) = fR;
% fT(1:3,4,:) = fO';
% fT(4,1:4,:) = repmat([0, 0, 0, 1], [1,1,nF]);
% tibl.fT = fT;
% 
% tibr.fR = quat2rotm(tibr.fQ);
% fR = tibr.fR;
% fO = repmat([0, -0.2, 0.4] , [nF,1]);
% fT(1:3,1:3,:) = fR;
% fT(1:3,4,:) = fO';
% fT(4,1:4,:) = repmat([0, 0, 0, 1], [1,1,nF]);
% tibr.fT = fT;
% 
% toel.fR = quat2rotm(toel.fQ);
% fR = toel.fR;
% fO = repmat([0, 0.2, 0.1] , [nF,1]);
% fT(1:3,1:3,:) = fR;
% fT(1:3,4,:) = fO';
% fT(4,1:4,:) = repmat([0, 0, 0, 1], [1,1,nF]);
% toel.fT = fT;
% 
% toer.fR = quat2rotm(toer.fQ);
% fR = toer.fR;
% fO = repmat([0, -0.2, 0.1] , [nF,1]);
% fT(1:3,1:3,:) = fR;
% fT(1:3,4,:) = fO';
% fT(4,1:4,:) = repmat([0, 0, 0, 1], [1,1,nF]);
% toer.fT = fT;
% 
% % plot
% figure;
% hold on; box on; grid on; grid minor;
% plotH(0.25, eye(4), [0.95 0.4 0.2]);
% plotH(0.15, tor.fT(:,:,RF), [0.2 0.6 0.95]);
% plotH(0.15, pel.fT(:,:,RF), [0.4 0.6 0.95]);
% plotH(0.15, feml.fT(:,:,RF), [0.95 0.2 0.4]);
% plotH(0.15, tibl.fT(:,:,RF), [0.95 0.3 0.5]);
% plotH(0.15, toel.fT(:,:,RF), [0.95 0.4 0.5]);
% plotH(0.15, femr.fT(:,:,RF), [0.2 0.95 0.55]);
% plotH(0.15, tibr.fT(:,:,RF), [0.4 0.85 0.55]);
% plotH(0.15, toer.fT(:,:,RF), [0.6 0.85 0.55]);
% axis equal;xlabel('X_{imufilter}');ylabel('Y_{imufilter}');zlabel('Z_{imufilter}');
% title('Orientation of sensors in CS0 (IMUfilter Kalman filter)');

%% Build the structure
sensors_data.torso = tor;
sensors_data.pelvis = pel;
sensors_data.femur_l = feml;
sensors_data.femur_r = femr;
sensors_data.tibia_l= tibl;
sensors_data.tibia_r = tibr;
sensors_data.calcn_l = call;
sensors_data.calcn_r = calr;
sensors_data.humerus_l= huml;
sensors_data.humerus_r = humr;
sensors_data.radius_l = radl;
sensors_data.radius_r = radr;
end