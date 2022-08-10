function [angle, q1, q2_rot] = OpenIMUs_AngAlign_v3(frange1, w1, w2, R1, R2, fs, visualization)
%% ------------------------ 
% frange1: range where hip AA movement is defined
% frange2: range where w1 and w2 have maximum  angular velocity
% w1: angular velocity (rad/s) of thigh sensor (or shank)
% w2: angular velocity (rad/s) of shank sensor (or foot)
% R1:
% R2:
% angle:
corr_thr = 0.85;
rmse_thr = 10;
[bLow,aLow] = butter(2, 1.2465*1/fs*2, 'low');
%% RF alignemnt  (CS 0 to the same cs 0) (e.g. thigh and shank)
% AA range check plot
% SENSOR 1
% Definition of the formula terms. Favre et al.2006
w_s = w1(frange1,:);% angular velocity
Rs_0 = R1;% rotation matrix of the sensor (orientation registered)
% Trasform w from s to 0
nF = size(w_s,1);% frame sample
w_0 = nan(nF,3);% init w0
for k=1:nF
    w_0 (k,:) = ( Rs_0(:,:,k) * w_s(k,:)' )'; % relation with the CS0
end
w_0_p = w_0(:,1:2); % projection on the x y plane
w_0_p = filtfilt(bLow, aLow, w_0_p);

w_0_p1 = w_0_p;
% figure;plot(w_0_p1);



[m , n] = mvarray(w_0_p); % module and versor of a vector
m = smooth(m);%  smooths the response data in column vector () using a moving average filter
% frange2 = 34:54; % ho visto che la parte utile di w e' in questo range
% frange2 = 12:40;
% figure;
% figure;plot(m)
% findpeaks(w_0_p(:,2), 1 ,'MinPeakHeight',0.35);
[vals,locs] = findpeaks(w_0_p(:,1), 1 ,'MinPeakHeight', 0.35);
frange2 = locs(find(vals == max(vals))) - 5 :1: locs(find(vals == max(vals))) +5;
[~ , n] = mvarray(w_0_p(frange2,:));
xaxis1 = mean(n,1); 
%%
% SENSOR2

w_s = w2(frange1,:);
Rs_0 = R2;
% Rs_0 = tib.R_fQ_rot;
% Trasforma w da s a 0
nF = size(w_s,1);
w_0 = nan(nF,3);
for k=1:nF
    w_0 (k,:) = ( Rs_0(:,:,k) * w_s(k,:)' )';   
end
w_0_p = w_0(:,1:2); % proiezione su piano x y
w_0_p = filtfilt(bLow, aLow, w_0_p);
% figure;plot(w_0_p);

[m , n] = mvarray(w_0_p); % module and versor of a vector
m = smooth(m);%  smooths the response data in column vector () using a moving average filter

[~ , n] = mvarray(w_0_p(frange2,:)); % uso lo stesso range anche per questo sensore
xaxis2 = mean(n,1); 
% tibl.PC = pca(w_0_p(frange2,:));


% l'angolo tra i due vettori e' 
angle = rad2deg(acos( dot(xaxis1, xaxis2, 2) ));

% angle = acos( dot(feml.PC(:,1)', tibl.PC(:,1)',2) ) *180/pi 
%%
%POSITIVE
q1 = quaternion(rotm2quat(R1));
q1_0 = q1.*conj(q1(1));
q2 = quaternion(rotm2quat(R2));
q2_0 = q2.*conj(q2(1));

qz = quaternion([angle, 0, 0],'eulerd','ZXY','frame');    
q2_rot = qz.*q2.*conj(q2(1)).*q2(1);
q2_rot_0 = q2_rot.*conj(q2_rot(1));
q2_rot = qz.*q2;

aw1 = angvel(q1, 1/fs, 'frame');% angular velocity
aw1(1,:) = aw1(2,:);
aw1(end,:) = aw1(end-1,:);
aw1_f = filtfilt(bLow, aLow, aw1);

aw2 = angvel(q2, 1/fs, 'frame');% angular velocity
aw2(1,:) = aw2(2,:);
aw2(end,:) = aw2(end-1,:);
aw2_f = filtfilt(bLow, aLow, aw2);

aw2_rot = angvel(q2_rot, 1/fs, 'frame');% angular velocity
aw2_rot(1,:) = aw2_rot(2,:);
aw2_rot(end,:) = aw2_rot(end-1,:);
aw2_rot_f = filtfilt(bLow, aLow, aw2_rot);

corr_aP = (corr(aw1_f(frange1,2), aw2_rot_f(frange1,2), 'Type', 'Pearson'));
figure;subplot(1,3,1);plot(aw1_f(frange1,2));hold on;plot(aw2_rot_f(frange1,2));title('pos');
rP = rmse(aw1_f(frange1,2), aw2_rot_f(frange1,2))*180/pi;
    
%NEGATIVE
q1 = quaternion(rotm2quat(R1));
q1_0 = q1.*conj(q1(1));
q2 = quaternion(rotm2quat(R2));
q2_0 = q2.*conj(q2(1));

qz = quaternion([-angle, 0, 0],'eulerd','ZXY','frame');    
q2_rot = qz.*q2.*conj(q2(1)).*q2(1);
q2_rot_0 = q2_rot.*conj(q2_rot(1));
q2_rot = qz.*q2;


aw1 = angvel(q1, 1/fs, 'frame');% angular velocity
aw1(1,:) = aw1(2,:);
aw1(end,:) = aw1(end-1,:);
aw1_f = filtfilt(bLow, aLow, aw1);

aw2 = angvel(q2, 1/fs, 'frame');% angular velocity
aw2(1,:) = aw2(2,:);
aw2(end,:) = aw2(end-1,:);
aw2_f = filtfilt(bLow, aLow, aw2);

aw2_rot = angvel(q2_rot, 1/fs, 'frame');% angular velocity
aw2_rot(1,:) = aw2_rot(2,:);
aw2_rot(end,:) = aw2_rot(end-1,:);
aw2_rot_f = filtfilt(bLow, aLow, aw2_rot);

corr_aN = (corr(aw1_f(frange1,2), aw2_rot_f(frange1,2), 'Type', 'Pearson'));
subplot(1,3,2);plot(aw1_f(frange1,2));hold on;plot(aw2_rot_f(frange1,2));title('neg');
rN = rmse(aw1_f(frange1,2), aw2_rot_f(frange1,2))*180/pi;


%ZERO
q1 = quaternion(rotm2quat(R1));
q1_0 = q1.*conj(q1(1));
q2 = quaternion(rotm2quat(R2));
q2_0 = q2.*conj(q2(1));

qz = quaternion([0, 0, 0],'eulerd','ZXY','frame');    
q2_rot = qz.*q2.*conj(q2(1)).*q2(1);
q2_rot_0 = q2_rot.*conj(q2_rot(1));
q2_rot = qz.*q2;


aw1 = angvel(q1, 1/fs, 'frame');% angular velocity
aw1(1,:) = aw1(2,:);
aw1(end,:) = aw1(end-1,:);
aw1_f = filtfilt(bLow, aLow, aw1);

aw2 = angvel(q2, 1/fs, 'frame');% angular velocity
aw2(1,:) = aw2(2,:);
aw2(end,:) = aw2(end-1,:);
aw2_f = filtfilt(bLow, aLow, aw2);

aw2_rot = angvel(q2_rot, 1/fs, 'frame');% angular velocity
aw2_rot(1,:) = aw2_rot(2,:);
aw2_rot(end,:) = aw2_rot(end-1,:);
aw2_rot_f = filtfilt(bLow, aLow, aw2_rot);

corr_a0 = (corr(aw1_f(frange1,2), aw2_rot_f(frange1,2), 'Type', 'Pearson'));
subplot(1,3,3);plot(aw1_f(frange1,2));hold on;plot(aw2_rot_f(frange1,2));title('zero');
r0 = rmse(aw1_f(frange1,2), aw2_rot_f(frange1,2))*180/pi;

  %%  
% CONDITIONS
%

if corr_aP > corr_thr  && rP < rmse_thr

        q1 = quaternion(rotm2quat(R1));
        q1_0 = q1.*conj(q1(1));
        q2 = quaternion(rotm2quat(R2));
        q2_0 = q2.*conj(q2(1));

        qz = quaternion([angle, 0, 0],'eulerd','ZXY','frame');    
        q2_rot = qz.*q2.*conj(q2(1)).*q2(1);
        q2_rot_0 = q2_rot.*conj(q2_rot(1));
        q2_rot = qz.*q2;


        aw1 = angvel(q1, 1/fs, 'frame');% angular velocity
        aw1(1,:) = aw1(2,:);
        aw1(end,:) = aw1(end-1,:);
        aw1_f = filtfilt(bLow, aLow, aw1);

        aw2 = angvel(q2, 1/fs, 'frame');% angular velocity
        aw2(1,:) = aw2(2,:);
        aw2(end,:) = aw2(end-1,:);
        aw2_f = filtfilt(bLow, aLow, aw2);

        aw2_rot = angvel(q2_rot, 1/fs, 'frame');% angular velocity
        aw2_rot(1,:) = aw2_rot(2,:);
        aw2_rot(end,:) = aw2_rot(end-1,:);
        aw2_rot_f = filtfilt(bLow, aLow, aw2_rot);
        
%         rP_new = rmse(aw1_f(frange1,2), aw2_rot_f(frange1,2))*180/pi
%         figure;plot(aw1_f(frange1,2));hold on;plot(aw2_rot_f(frange1,2));
        
end


if corr_aN > corr_thr && rN < rmse_thr
    angle = -angle;
    q1 = quaternion(rotm2quat(R1));
    q1_0 = q1.*conj(q1(1));
    q2 = quaternion(rotm2quat(R2));
    q2_0 = q2.*conj(q2(1));

    qz = quaternion([angle, 0, 0],'eulerd','ZXY','frame');    
    q2_rot = qz.*q2.*conj(q2(1)).*q2(1);
    q2_rot_0 = q2_rot.*conj(q2_rot(1));
    q2_rot = qz.*q2;


    aw1 = angvel(q1, 1/fs, 'frame');% angular velocity
    aw1(1,:) = aw1(2,:);
    aw1(end,:) = aw1(end-1,:);
    aw1_f = filtfilt(bLow, aLow, aw1);

    aw2 = angvel(q2, 1/fs, 'frame');% angular velocity
    aw2(1,:) = aw2(2,:);
    aw2(end,:) = aw2(end-1,:);
    aw2_f = filtfilt(bLow, aLow, aw2);

    aw2_rot = angvel(q2_rot, 1/fs, 'frame');% angular velocity
    aw2_rot(1,:) = aw2_rot(2,:);
    aw2_rot(end,:) = aw2_rot(end-1,:);
    aw2_rot_f = filtfilt(bLow, aLow, aw2_rot);
end

if corr_a0 > corr_thr && r0 < rmse_thr

     angle = 0;

    q1 = quaternion(rotm2quat(R1));
    q1_0 = q1.*conj(q1(1));
    q2 = quaternion(rotm2quat(R2));
    q2_0 = q2.*conj(q2(1));

    qz = quaternion([angle, 0, 0],'eulerd','ZXY','frame');    
    q2_rot = qz.*q2.*conj(q2(1)).*q2(1);
    q2_rot_0 = q2_rot.*conj(q2_rot(1));
    q2_rot = qz.*q2;


    aw1 = angvel(q1, 1/fs, 'frame');% angular velocity
    aw1(1,:) = aw1(2,:);
    aw1(end,:) = aw1(end-1,:);
    aw1_f = filtfilt(bLow, aLow, aw1);

    aw2 = angvel(q2, 1/fs, 'frame');% angular velocity
    aw2(1,:) = aw2(2,:);
    aw2(end,:) = aw2(end-1,:);
    aw2_f = filtfilt(bLow, aLow, aw2);

    aw2_rot = angvel(q2_rot, 1/fs, 'frame');% angular velocity
    aw2_rot(1,:) = aw2_rot(2,:);
    aw2_rot(end,:) = aw2_rot(end-1,:);
    aw2_rot_f = filtfilt(bLow, aLow, aw2_rot);

end
%%
corr_max = max([corr_aP, corr_aN, corr_a0]);
rmse_min = min([rP, rN, r0]);
if corr_max <= corr_thr || rmse_min >= rmse_thr
    if rmse_min == rP
        angle = angle;
    end
    if rmse_min == rN
        angle = -angle;
    end
    if rmse_min == r0
        angle = 0;
    end
    
    q1 = quaternion(rotm2quat(R1));
    q1_0 = q1.*conj(q1(1));
    q2 = quaternion(rotm2quat(R2));
    q2_0 = q2.*conj(q2(1));

    qz = quaternion([angle, 0, 0],'eulerd','ZXY','frame');    
    q2_rot = qz.*q2.*conj(q2(1)).*q2(1);
    q2_rot_0 = q2_rot.*conj(q2_rot(1));
    q2_rot = qz.*q2;


    aw1 = angvel(q1, 1/fs, 'frame');% angular velocity
    aw1(1,:) = aw1(2,:);
    aw1(end,:) = aw1(end-1,:);
    aw1_f = filtfilt(bLow, aLow, aw1);

    aw2 = angvel(q2, 1/fs, 'frame');% angular velocity
    aw2(1,:) = aw2(2,:);
    aw2(end,:) = aw2(end-1,:);
    aw2_f = filtfilt(bLow, aLow, aw2);

    aw2_rot = angvel(q2_rot, 1/fs, 'frame');% angular velocity
    aw2_rot(1,:) = aw2_rot(2,:);
    aw2_rot(end,:) = aw2_rot(end-1,:);
    aw2_rot_f = filtfilt(bLow, aLow, aw2_rot);
    
end
    
    
    


    disp(['AngleP-> r: ' num2str(corr_aP) ' --- rmse: ' num2str(rP)]);
    disp(['AngleN-> r: ' num2str(corr_aN) ' --- rmse: ' num2str(rN)]);
    disp(['Angle0-> r: ' num2str(corr_a0) ' --- rmse: ' num2str(r0)]);
    
  
    % Plot
    figure;
    subplot(3,2,1);plot(aw1_f(frange1,1)); hold on;plot(aw2_f(frange1,1));grid on;
    title('Sensors NOT aligned');
    subplot(3,2,3);plot(aw1_f(frange1,2)); hold on;plot(aw2_f(frange1,2));grid on;
    subplot(3,2,5);plot(aw1_f(frange1,3)); hold on;plot(aw2_f(frange1,3));grid on;
    
    subplot(3,2,2);plot(aw1_f(frange1,1)); hold on;plot(aw2_rot_f(frange1,1));grid on;
    title('Sensors aligned');
    subplot(3,2,4);plot(aw1_f(frange1,2)); hold on;plot(aw2_rot_f(frange1,2));grid on;
    subplot(3,2,6);plot(aw1_f(frange1,3)); hold on;plot(aw2_rot_f(frange1,3));grid on;

    
    disp(['Sensors Aligned ---> ', num2str(angle),' deg']);


   %% 
    
if visualization == "true"   
    viewer = HelperOrientationViewer('Title', {'Sensors NOT aligned'});
    for ii=frange1
        viewer(q1(ii,:));
        pause(0.01);
    end
    pause(1.5)
    for ii=frange1
        viewer(q2(ii,:));
        pause(0.01);
    end
    pause(2);
     
    viewer = HelperOrientationViewer('Title', {'Sensors aligned'});
    for ii=frange1
        viewer(q1(ii,:));
        pause(0.01);
    end
    pause(1.5)
    for ii=frange1
        viewer(q2_rot(ii,:));
        pause(0.01);
    end
    pause(2);
        
%     close all
%     figure;
%     subplot(3,1,1);plot(D(:,1), 'k', 'LineWidth', 2);
%     grid on;title('IE');
%     subplot(3,1,2);plot(D(:,2), 'k', 'LineWidth', 2);
%     grid on;title('FE');
%     subplot(3,1,3);plot(D(:,3), 'k', 'LineWidth', 2);
%     grid on;title('AA');
else
    disp('No sensors visual')   
    
end

    disp(' ')



end