clc;
clear;
close all;
%% LOAD and Create Sensor Structfs 
fs = 60;
% names = {'d1', 'd2', 'd3', 'd4','d5','d6','d7','d8','d9','d10','d11','d12','d13','d14','d15'};
% names = {'d1', 'd2', 'd3', 'd4','d5','d6','d7','d8','d9','d10','d11','d12','d13','d14','d15','d16','d17','d18','d19','d20','d21','d22','d23'};
% 
names = {'d1','d2', 'd3', 'd4','d5','d6','d7','d8','d9','d10','d11','d12','d13','d14','d15','d16','d17','d18','d19'};
% names = {'d11','d12','d13'};
DS = struct();
for i = 1:length(names)
    if i<10
        load(['IMU_00', num2str(i), '_v1']);
%         load(['Session1-00', num2str(i),'_DataM']);
    else
        load(['IMU_0', num2str(i), '_v1']);
%         load(['Session1-0', num2str(i),'_DataM']);
    end
    DS.(names{i}) =  create_sensors_struct_MVN3(fs, a, m, q);
end
%% SAVE DATA
save('Data_Struct_S01', 'DS');
%% Movement range
%5) Walk1
close all;figure;
plot(DS.d7.femur_r.g(:,2));hold on;
plot(DS.d7.femur_l.g(:,2));
plot(DS.d7.pelvis.g(:,1));grid on;


DS.d5.standing_range = 1:100;
DS.d5.sit_range = 100:1500;
DS.d5.adduction_range_r = 1740:1920;
DS.d5.adduction_range_l = 2580:2760;
DS.d5.walking_range =4080:4320;
DS.d5.trunk_rot_range = 3900:4080;
%6) Walk2
DS.d6.standing_range = 1:80;
DS.d6.sit_range = 400:1200;
DS.d6.adduction_range_r = 1450:1600;
DS.d6.adduction_range_l = 2200:2400;
DS.d6.walking_range =3250:3400;
DS.d6.trunk_rot_range = 3000:3100;
%7)Walk3
DS.d7.standing_range = 1:387;
DS.d7.sit_range = 387:1080;
DS.d7.adduction_range_r = 1419:1570; 
DS.d7.adduction_range_l = 2300:2420; 
DS.d7.walking_range =3200:3360;
DS.d7.trunk_rot_range = 3000:3200;

