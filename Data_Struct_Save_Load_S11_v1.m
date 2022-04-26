clc;
clear;
close all;
%% LOAD and Create Sensor Structfs 
fs = 60;
% names = {'d1', 'd2', 'd3', 'd4','d5','d6','d7','d8','d9','d10','d11','d12','d13','d14','d15'};
names = {'d1', 'd2', 'd3', 'd4','d5','d6','d7','d8','d9','d10','d11','d12','d13','d14','d15','d16','d17',...
    'd18','d19','d20','d21','d22','d23','d24','d25','d26','d27','d28'};
% 
% names = {'d1', 'd2', 'd3', 'd4','d5','d6','d7','d8','d9','d10','d11','d12','d13','d14','d15','d16','d17','d18','d19'};
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
save('Data_Struct_S11', 'DS');
%% Movement range
% 1) W1
trials = fieldnames(DS);
x = nan(7,length(names));
close all;
for i = 28:length(names)
    figure;
    trial = getfield(DS, trials{i});
    plot(trial.femur_r.g(:,2));hold on;
    plot(trial.femur_l.g(:,2));
    plot(trial.pelvis.g(:,1));grid on;
    [xt, y ] = ginput; x(:,i) = round(xt);
end

%% Range
% 1) X
DS.d1.standing_range = 1:x(1,1);
DS.d1.sit_range = x(1,1):x(2,1);
DS.d1.adduction_range_r = x(2,1):x(3,1); 
DS.d1.adduction_range_l = x(3,1):x(4,1); 
DS.d1.walking_range =x(5,1):x(6,1); 
DS.d1.trunk_rot_range =x(6,1):x(7,1); 
% 2) W2
DS.d2.standing_range = 1:x(1,2);
DS.d2.sit_range = x(1,2):x(2,2);
DS.d2.adduction_range_r = x(2,2):x(3,2); 
DS.d2.adduction_range_l = x(3,2):x(4,2); 
DS.d2.walking_range =x(5,2):x(6,2); 
DS.d2.trunk_rot_range =x(6,2):x(7,2); 
% 3) W3
DS.d3.standing_range = 1:x(1,3);
DS.d3.sit_range = x(1,3):x(2,3);
DS.d3.adduction_range_r = x(2,3):x(3,3); 
DS.d3.adduction_range_l = x(3,3):x(4,3); 
DS.d3.walking_range =x(5,3):x(6,3); 
DS.d3.trunk_rot_range =x(6,3):x(7,3); 
% 4) X
DS.d4.standing_range = 1:x(1,4);
DS.d4.sit_range = x(1,4):x(2,4);
DS.d4.adduction_range_r = x(2,4):x(3,4); 
DS.d4.adduction_range_l = x(3,4):x(4,4); 
DS.d4.walking_range =x(5,4):x(6,4); 
DS.d4.trunk_rot_range =x(6,4):x(7,4); 
% 5) W5
DS.d5.standing_range = 1:x(1,5);
DS.d5.sit_range = x(1,5):x(2,5);
DS.d5.adduction_range_r = x(2,5):x(3,5); 
DS.d5.adduction_range_l = x(3,5):x(4,5); 
DS.d5.walking_range =x(5,5):x(6,5); 
DS.d5.trunk_rot_range =x(6,5):x(7,5); 
% 6) X
DS.d6.standing_range = 1:x(1,6);
DS.d6.sit_range = x(1,6):x(2,6);
DS.d6.adduction_range_r = x(2,6):x(3,6); 
DS.d6.adduction_range_l = x(3,6):x(4,6); 
DS.d6.walking_range =x(5,6):x(6,6); 
DS.d6.trunk_rot_range =x(6,6):x(7,6); 
% 7) W7
DS.d7.standing_range = 1:x(1,7);
DS.d7.sit_range = x(1,7):x(2,7);
DS.d7.adduction_range_r = x(2,7):x(3,7); 
DS.d7.adduction_range_l = x(3,7):x(4,7); 
DS.d7.walking_range =x(5,7):x(6,7); 
DS.d7.trunk_rot_range =x(6,7):x(7,7); 
% 8) W8
DS.d8.standing_range = 1:x(1,8);
DS.d8.sit_range = x(1,8):x(2,8);
DS.d8.adduction_range_r = x(2,8):x(3,8); 
DS.d8.adduction_range_l = x(3,8):x(4,8); 
DS.d8.walking_range =x(5,8):x(6,8); 
DS.d8.trunk_rot_range =x(6,8):x(7,8); 
% 9) S1
DS.d9.standing_range = 1:x(1,9);
DS.d9.sit_range = x(1,9):x(2,9);
DS.d9.adduction_range_r = x(2,9):x(3,9); 
DS.d9.adduction_range_l = x(3,9):x(4,9); 
DS.d9.walking_range =x(5,9):x(6,9); 
DS.d9.trunk_rot_range =x(6,9):x(7,9); 
%10) S2
DS.d10.standing_range = 1:x(1,10);
DS.d10.sit_range = x(1,10):x(2,10);
DS.d10.adduction_range_r = x(2,10):x(3,10); 
DS.d10.adduction_range_l = x(3,10):x(4,10); 
DS.d10.walking_range =x(5,10):x(6,10); 
DS.d10.trunk_rot_range =x(6,10):x(7,10); 
%11) S3
DS.d11.standing_range = 1:x(1,11);
DS.d11.sit_range = x(1,11):x(2,11);
DS.d11.adduction_range_r = x(2,11):x(3,11); 
DS.d11.adduction_range_l = x(3,11):x(4,11); 
DS.d11.walking_range =x(5,11):x(6,11); 
DS.d11.trunk_rot_range =x(6,11):x(7,11); 
%12) S4
DS.d12.standing_range = 1:x(1,12);
DS.d12.sit_range = x(1,12):x(2,12);
DS.d12.adduction_range_r = x(2,12):x(3,12); 
DS.d12.adduction_range_l = x(3,12):x(4,12); 
DS.d12.walking_range =x(5,12):x(6,12); 
DS.d12.trunk_rot_range =x(6,12):x(7,12); 
%13)S5
DS.d13.standing_range = 1:x(1,13);
DS.d13.sit_range = x(1,13):x(2,13);
DS.d13.adduction_range_r = x(2,13):x(3,13); 
DS.d13.adduction_range_l = x(3,13):x(4,13); 
DS.d13.walking_range =x(5,13):x(6,13); 
DS.d13.trunk_rot_range =x(6,13):x(7,13); 
%14) R1
DS.d14.standing_range = 1:x(1,14);
DS.d14.sit_range = x(1,14):x(2,14);
DS.d14.adduction_range_r = x(2,14):x(3,14); 
DS.d14.adduction_range_l = x(3,14):x(4,14); 
DS.d14.walking_range =x(5,14):x(6,14); 
DS.d14.trunk_rot_range =x(6,14):x(7,14);   
%15) R2
DS.d15.standing_range = 1:x(1,15);
DS.d15.sit_range = x(1,15):x(2,15);
DS.d15.adduction_range_r = x(2,15):x(3,15); 
DS.d15.adduction_range_l = x(3,15):x(4,15); 
DS.d15.walking_range =x(5,15):x(6,15); 
DS.d15.trunk_rot_range =x(6,15):x(7,15); 
%16) R3
DS.d16.standing_range = 1:x(1,16);
DS.d16.sit_range = x(1,16):x(2,16);
DS.d16.adduction_range_r = x(2,16):x(3,16); 
DS.d16.adduction_range_l = x(3,16):x(4,16); 
DS.d16.walking_range =x(5,16):x(6,16); 
DS.d16.trunk_rot_range =x(6,16):x(7,16); 
%17) R4
DS.d17.standing_range = 1:x(1,17);
DS.d17.sit_range = x(1,17):x(2,17);
DS.d17.adduction_range_r = x(2,17):x(3,17); 
DS.d17.adduction_range_l = x(3,17):x(4,17); 
DS.d17.walking_range =x(5,17):x(6,17); 
DS.d17.trunk_rot_range =x(6,17):x(7,17); 
%18) R5
DS.d18.standing_range = 1:x(1,18);
DS.d18.sit_range = x(1,18):x(2,18);
DS.d18.adduction_range_r = x(2,18):x(3,18); 
DS.d18.adduction_range_l = x(3,18):x(4,18); 
DS.d18.walking_range =x(5,18):x(6,18); 
DS.d18.trunk_rot_range =x(6,18):x(7,18); 
%19) Npose1
DS.d19.standing_range = 1:x(1,19);
DS.d19.sit_range = x(1,19):x(2,19);
DS.d19.adduction_range_r = x(2,19):x(3,19); 
DS.d19.adduction_range_l = x(3,19):x(4,19); 
DS.d19.walking_range =x(5,19):x(6,19); 
DS.d19.trunk_rot_range =x(6,19):x(7,19); 
%20) Npose2
DS.d20.standing_range = 1:x(1,20);
DS.d20.sit_range = x(1,20):x(2,20);
DS.d20.adduction_range_r = x(2,20):x(3,20); 
DS.d20.adduction_range_l = x(3,20):x(4,20); 
DS.d20.walking_range =x(5,20):x(6,20); 
DS.d20.trunk_rot_range =x(6,20):x(7,20); 
%21) Npose3
DS.d21.standing_range = 1:x(1,21);
DS.d21.sit_range = x(1,21):x(2,21);
DS.d21.adduction_range_r = x(2,21):x(3,21); 
DS.d21.adduction_range_l = x(3,21):x(4,21); 
DS.d21.walking_range =x(5,21):x(6,21); 
DS.d21.trunk_rot_range =x(6,21):x(7,21); 
%22) X
DS.d22.standing_range = 1:x(1,22);
DS.d22.sit_range = x(1,22):x(2,22);
DS.d22.adduction_range_r = x(2,22):x(3,22); 
DS.d22.adduction_range_l = x(3,22):x(4,22); 
DS.d22.walking_range =x(5,22):x(6,22); 
DS.d22.trunk_rot_range =x(6,22):x(7,22); 
%23) X
DS.d23.standing_range = 1:x(1,22);
DS.d23.sit_range = x(1,22):x(2,22);
DS.d23.adduction_range_r = x(2,22):x(3,22); 
DS.d23.adduction_range_l = x(3,22):x(4,22); 
DS.d23.walking_range =x(5,22):x(6,22); 
DS.d23.trunk_rot_range =x(6,22):x(7,22); 
%24) Tpose1_new
DS.d24.standing_range = 1:x(1,22);
DS.d24.sit_range = x(1,22):x(2,22);
DS.d24.adduction_range_r = x(2,22):x(3,22); 
DS.d24.adduction_range_l = x(3,22):x(4,22); 
DS.d24.walking_range =x(5,22):x(6,22); 
DS.d24.trunk_rot_range =x(6,22):x(7,22); 
%25) Tpose1_new1
DS.d25.standing_range = 1:x(1,22);
DS.d25.sit_range = x(1,22):x(2,22);
DS.d25.adduction_range_r = x(2,22):x(3,22); 
DS.d25.adduction_range_l = x(3,22):x(4,22); 
DS.d25.walking_range =x(5,22):x(6,22); 
DS.d25.trunk_rot_range =x(6,22):x(7,22); 
%26) Tpose1_new2
DS.d26.standing_range = 1:x(1,22);
DS.d26.sit_range = x(1,22):x(2,22);
DS.d26.adduction_range_r = x(2,22):x(3,22); 
DS.d26.adduction_range_l = x(3,22):x(4,22); 
DS.d26.walking_range =x(5,22):x(6,22); 
DS.d26.trunk_rot_range =x(6,22):x(7,22); 
%27) ROM1
DS.d27.standing_range = 1:x(1,22);
DS.d27.sit_range = x(1,22):x(2,22);
DS.d27.adduction_range_r = x(2,22):x(3,22); 
DS.d27.adduction_range_l = x(3,22):x(4,22); 
DS.d27.walking_range =x(5,22):x(6,22); 
DS.d27.trunk_rot_range =x(6,22):x(7,22); 
%28) ROM2
DS.d28.standing_range = 1:x(1,22);
DS.d28.sit_range = x(1,22):x(2,22);
DS.d28.adduction_range_r = x(2,22):x(3,22); 
DS.d28.adduction_range_l = x(3,22):x(4,22); 
DS.d28.walking_range =x(5,22):x(6,22); 
DS.d28.trunk_rot_range =x(6,22):x(7,22); 