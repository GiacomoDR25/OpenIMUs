clc;
%% Coord_Model
import org.opensim.modeling.*
OutPath =pwd;
model = Model(modelFileName); 

% Model position
Coord = struct();
Coord_Est = struct();
%% READ COORD VALUES

% Shoulder R
armR = model.updJointSet().get('acromial_r');
Coord.arm_r.flexion = armR.get_coordinates(0).getDefaultValue();
Coord.arm_r.adduction = armR.get_coordinates(1).getDefaultValue();
Coord.arm_r.rotation = armR.get_coordinates(2).getDefaultValue();

% Shoulder L
armL = model.updJointSet().get('acromial_l');
Coord.arm_l.flexion = armL.get_coordinates(0).getDefaultValue();
Coord.arm_l.adduction = armL.get_coordinates(1).getDefaultValue();
Coord.arm_l.rotation = armL.get_coordinates(2).getDefaultValue();

% Elbow R
elbR = model.updJointSet().get('elbow_r');
Coord.elbow_r.flexion = elbR.get_coordinates(0).getDefaultValue();
elbR2 = model.updJointSet().get('radioulnar_r');
Coord.pro_sup_r.rotation = elbR2.get_coordinates(0).getDefaultValue();


% Elbow L
elbL = model.updJointSet().get('elbow_l');
Coord.elbow_l.flexion = elbL.get_coordinates(0).getDefaultValue();
elbL2 = model.updJointSet().get('radioulnar_l');
Coord.pro_sup_l.rotation = elbL2.get_coordinates(0).getDefaultValue();


%Torso
t = model.updJointSet().get('back');
t_FE = t.get_coordinates(0);
t_AA = t.get_coordinates(1);
t_IE = t.get_coordinates(2);
Coord.lumbar.extension = t_FE.getDefaultValue();
Coord.lumbar.bending = t_AA.getDefaultValue();
Coord.lumbar.rotation = t_IE.getDefaultValue();

%Pelvis
pl = model.updJointSet().get('ground_pelvis');
p_FE = pl.get_coordinates(0);
p_AA = pl.get_coordinates(1);
p_IE = pl.get_coordinates(2);
Coord.pelvis.tilt = p_FE.getDefaultValue();
Coord.pelvis.list = p_AA.getDefaultValue();
Coord.pelvis.rotation = p_IE.getDefaultValue();

%Hip R
hR = model.updJointSet().get('hip_r');
Coord.hip_r.flexion = hR.get_coordinates(0).getDefaultValue();
Coord.hip_r.adduction = hR.get_coordinates(1).getDefaultValue();
Coord.hip_r.rotation = hR.get_coordinates(2).getDefaultValue();
%Hip L
hL = model.updJointSet().get('hip_l');
Coord.hip_l.flexion = hL.get_coordinates(0).getDefaultValue();
Coord.hip_l.adduction = hL.get_coordinates(1).getDefaultValue();
Coord.hip_l.rotation = hL.get_coordinates(2).getDefaultValue();
%Knee R
kR = model.updJointSet().get('knee_r');
Coord.knee_r.angle = kR.get_coordinates(0).getDefaultValue();
Coord.knee_r.rotation = kR.get_coordinates(1).getDefaultValue();
Coord.knee_r.adduction = kR.get_coordinates(2).getDefaultValue();
%KneeL
kL = model.updJointSet().get('knee_l');
Coord.knee_l.angle = kL.get_coordinates(0).getDefaultValue();
Coord.knee_l.rotation = kL.get_coordinates(1).getDefaultValue();
Coord.knee_l.adduction = kL.get_coordinates(2).getDefaultValue();
%AnkleR
aR = model.updJointSet().get('ankle_r');
Coord.ankle_r.angle = aR.get_coordinates(0).getDefaultValue();
%AnkleL
aL = model.updJointSet().get('ankle_l');
Coord.ankle_l.angle = aL.get_coordinates(0).getDefaultValue();
%SubtalarR
sR = model.updJointSet().get('subtalar_r');
Coord.subtalar_r.angle = sR.get_coordinates(0).getDefaultValue();

%SubtalarL
sL = model.updJointSet().get('subtalar_l');
Coord.subtalar_l.angle = sL.get_coordinates(0).getDefaultValue();


%% WRITE COORD EST VALUES
if coord_estimation
    %Pelvis
    pl = model.updJointSet().get('ground_pelvis');
    p_FE = pl.upd_coordinates(0);
    p_AA = pl.upd_coordinates(1);
    p_IE = pl.upd_coordinates(2);
    p_FE.setDefaultValue(deg2rad(pelvis_tilt));
    p_AA.setDefaultValue(deg2rad(pelvis_list));
    p_IE.setDefaultValue(deg2rad(pelvis_rotation));
    Coord_Est.pelvis.tilt = p_FE.getDefaultValue();
    Coord_Est.pelvis.list = p_AA.getDefaultValue();
    Coord_Est.pelvis.rotation = p_IE.getDefaultValue();
    %Torso
    t = model.updJointSet().get('back');
    t_FE = t.upd_coordinates(0);
    t_AA = t.upd_coordinates(1);
    t_IE = t.upd_coordinates(2);
    t_FE.setDefaultValue(deg2rad(lumbar_extension));
    t_AA.setDefaultValue(deg2rad(lumbar_bending));
    t_IE.setDefaultValue(deg2rad(lumbar_rotation));
    Coord_Est.lumbar.extension = t_FE.getDefaultValue();
    Coord_Est.lumbar.bending = t_AA.getDefaultValue();
    Coord_Est.lumbar.rotation = t_IE.getDefaultValue();
    %Hip R
    hR = model.updJointSet().get('hip_r');
    hR.upd_coordinates(0).setDefaultValue( deg2rad(hip_flexion_r));
    hR.upd_coordinates(1).setDefaultValue(deg2rad( hip_adduction_r ));
    hR.upd_coordinates(2).setDefaultValue(deg2rad(hip_rotation_r));
    Coord_Est.hip_r.flexion = hR.upd_coordinates(0).getDefaultValue();
    Coord_Est.hip_r.adduction = hR.upd_coordinates(1).getDefaultValue();
    Coord_Est.hip_r.rotation = hR.upd_coordinates(2).getDefaultValue();
    %Hip L
    hL = model.updJointSet().get('hip_l');
    hL.upd_coordinates(0).setDefaultValue( deg2rad(hip_flexion_l) );
    hL.upd_coordinates(1).setDefaultValue(deg2rad(hip_adduction_l ));
    hL.upd_coordinates(2).setDefaultValue(deg2rad( hip_rotation_l ));
    Coord_Est.hip_l.flexion = hL.upd_coordinates(0).getDefaultValue();
    Coord_Est.hip_l.adduction = hL.upd_coordinates(1).getDefaultValue();
    Coord_Est.hip_l.rotation = hL.upd_coordinates(2).getDefaultValue();
    %Knee R
    kR = model.updJointSet().get('knee_r');
    kR.upd_coordinates(0).setDefaultValue(deg2rad(knee_angle_r));
    kR.upd_coordinates(1).setDefaultValue(deg2rad(knee_rotation_r));
    kR.upd_coordinates(2).setDefaultValue(deg2rad(knee_adduction_r));
    Coord_Est.knee_r.angle = kR.upd_coordinates(0).getDefaultValue();
    Coord_Est.knee_r.rotation = kR.upd_coordinates(1).getDefaultValue();
    Coord_Est.knee_r.adduction = kR.upd_coordinates(2).getDefaultValue();
    %KneeL
    kL = model.updJointSet().get('knee_l');
    kL.upd_coordinates(0).setDefaultValue(deg2rad(knee_angle_l));
    kL.upd_coordinates(1).setDefaultValue(deg2rad(knee_rotation_l));
    kL.upd_coordinates(2).setDefaultValue(deg2rad(knee_adduction_l));
    Coord_Est.knee_l.angle = kL.upd_coordinates(0).getDefaultValue();
    Coord_Est.knee_l.rotation = kL.upd_coordinates(1).getDefaultValue();
    Coord_Est.knee_l.adduction = kL.upd_coordinates(2).getDefaultValue();
    %AnkleR
    aR = model.updJointSet().get('ankle_r');
    aR.upd_coordinates(0).setDefaultValue(deg2rad(ankle_angle_r));
    Coord_Est.ankle_r.angle = aR.upd_coordinates(0).getDefaultValue();
    %AnkleL
    aL = model.updJointSet().get('ankle_l');
    Coord_Est.ankle_l.angle = aL.upd_coordinates(0).getDefaultValue();
    aL.upd_coordinates(0).setDefaultValue(deg2rad(ankle_angle_l));
    %SubtalarR
    sR = model.updJointSet().get('subtalar_r');
    sR.upd_coordinates(0).setDefaultValue(deg2rad(subtalar_angle_r));
    Coord_Est.subtalar_r.angle = sR.upd_coordinates(0).getDefaultValue();
    %SubtalarL
    sL = model.updJointSet().get('subtalar_l');
    sL.upd_coordinates(0).setDefaultValue(deg2rad(subtalar_angle_l));
    Coord_Est.subtalar_l.angle = sL.upd_coordinates(0).getDefaultValue();
end


%% CREATE BODYSET MODEL

bodySet = model.getBodySet();
% ------- Body segments ---------
humerus_r_body = bodySet.get('humerus_r');
radius_r_body = bodySet.get('radius_r');
humerus_l_body = bodySet.get('humerus_l');
radius_l_body = bodySet.get('radius_l');
torso_body = bodySet.get('torso');
pelvis_body = bodySet.get('pelvis');
femur_r_body = bodySet.get('femur_r');
tibia_r_body = bodySet.get('tibia_r');
calcn_r_body = bodySet.get('calcn_r');
femur_l_body = bodySet.get('femur_l');
tibia_l_body = bodySet.get('tibia_l');
calcn_l_body = bodySet.get('calcn_l');
% ------ IMU attached to the body segment ----------
humerus_r_imu = PhysicalOffsetFrame.safeDownCast(humerus_r_body.findComponent('humerus_r_imu'));
radius_r_imu = PhysicalOffsetFrame.safeDownCast(radius_r_body.findComponent('radius_r_imu'));
humerus_l_imu = PhysicalOffsetFrame.safeDownCast(humerus_l_body.findComponent('humerus_l_imu'));
radius_l_imu = PhysicalOffsetFrame.safeDownCast(radius_l_body.findComponent('radius_l_imu'));
torso_imu = PhysicalOffsetFrame.safeDownCast(torso_body.findComponent('torso_imu'));
pelvis_imu = PhysicalOffsetFrame.safeDownCast(pelvis_body.findComponent('pelvis_imu'));
femur_r_imu = PhysicalOffsetFrame.safeDownCast(femur_r_body.findComponent('femur_r_imu'));
tibia_r_imu = PhysicalOffsetFrame.safeDownCast(tibia_r_body.findComponent('tibia_r_imu'));
calcn_r_imu = PhysicalOffsetFrame.safeDownCast(calcn_r_body.findComponent('calcn_r_imu'));
femur_l_imu = PhysicalOffsetFrame.safeDownCast(femur_l_body.findComponent('femur_l_imu'));
tibia_l_imu = PhysicalOffsetFrame.safeDownCast(tibia_l_body.findComponent('tibia_l_imu'));
calcn_l_imu = PhysicalOffsetFrame.safeDownCast(calcn_l_body.findComponent('calcn_l_imu'));

%% ------ Read the orientation/translation --------
disp(['----- humerus_r_imu -----', char(13), ...
    'tr: [', num2str(humerus_r_imu.get_translation.getAsMat'),']'...
char(13), 'or: [', num2str(humerus_r_imu.get_orientation.getAsMat') , ']']);
disp(['----- radius_r_imu -----', char(13), ...
    'tr: [', num2str(radius_r_imu.get_translation.getAsMat'),']'...
char(13), 'or: [', num2str(radius_r_imu.get_orientation.getAsMat') , ']']);
disp(['----- humerus_l_imu -----', char(13), ...
    'tr: [', num2str(humerus_l_imu.get_translation.getAsMat'),']'...
char(13), 'or: [', num2str(humerus_l_imu.get_orientation.getAsMat') , ']']);
disp(['----- radius_l_imu -----', char(13), ...
    'tr: [', num2str(radius_l_imu.get_translation.getAsMat'),']'...
char(13), 'or: [', num2str(radius_l_imu.get_orientation.getAsMat') , ']']);
disp(['----- torso_imu -----', char(13), ...
    'tr: [', num2str(torso_imu.get_translation.getAsMat'),']'...
char(13), 'or: [', num2str(torso_imu.get_orientation.getAsMat') , ']']);
disp(['----- pelvis_imu -----', char(13), ...
    'tr: [', num2str(pelvis_imu.get_translation.getAsMat'),']'...
char(13), 'or: [', num2str(pelvis_imu.get_orientation.getAsMat') , ']']);
disp(['----- femur_r_imu -----', char(13), ...
    'tr: [', num2str(femur_r_imu.get_translation.getAsMat'),']'...
char(13), 'or: [', num2str(femur_r_imu.get_orientation.getAsMat') , ']']);
disp(['----- tibia_r_imu -----' , char(13), ...
    'tr: [', num2str(tibia_r_imu.get_translation.getAsMat'),']'...
char(13), 'or: [', num2str(tibia_r_imu.get_orientation.getAsMat') , ']']);
disp(['----- calcn_r_imu -----', char(13), ...
    'tr: [', num2str(calcn_r_imu.get_translation.getAsMat'),']'...
char(13), 'or: [', num2str(calcn_r_imu.get_orientation.getAsMat') , ']']);
disp(['----- femur_l_imu -----', char(13), ...
    'tr: [', num2str(femur_l_imu.get_translation.getAsMat'),']'...
char(13), 'or: [', num2str(femur_l_imu.get_orientation.getAsMat') , ']']);
disp(['----- tibia_l_imu -----', char(13), ...
    'tr: [', num2str(tibia_l_imu.get_translation.getAsMat'),']'...
char(13), 'or: [', num2str(tibia_l_imu.get_orientation.getAsMat') , ']']);
disp(['----- calcn_l_imu -----', char(13), ...
    'tr: [', num2str(calcn_l_imu.get_translation.getAsMat'),']'...
char(13), 'or: [', num2str(calcn_l_imu.get_orientation.getAsMat') , ']']);
%% Reset the IMU model Orientation

humerus_r_imu.set_orientation(Vec3(0));
radius_r_imu.set_orientation(Vec3(0));
humerus_l_imu.set_orientation(Vec3(0));
radius_l_imu.set_orientation(Vec3(0));
torso_imu.set_orientation(Vec3(0));
pelvis_imu.set_orientation(Vec3(0));
femur_r_imu.set_orientation(Vec3(0));
tibia_r_imu.set_orientation(Vec3(0));
calcn_r_imu.set_orientation(Vec3(0));
femur_l_imu.set_orientation(Vec3(0));
tibia_l_imu.set_orientation(Vec3(0));
calcn_l_imu.set_orientation(Vec3(0));

%% ------------ Check IMU orientation --------------
if visualizeIMU
    viewer =   HelperOrientationViewer;
    viewer(quaternion(humerus_r_imu.get_orientation.getAsMat', 'euler', 'XYZ', 'frame'));title('humerus R imu');% humerus_r_imu
    pause(1.5)
    viewer(quaternion(radius_r_imu.get_orientation.getAsMat', 'euler', 'XYZ', 'frame'));title('radius R imu');% radius_r_imu
    pause(1.5)
    viewer(quaternion(humerus_l_imu.get_orientation.getAsMat', 'euler', 'XYZ', 'frame'));title('humerus L imu');% humerus_l_imu
    pause(1.5)
    viewer(quaternion(radius_l_imu.get_orientation.getAsMat', 'euler', 'XYZ', 'frame'));title('radius L imu');% radius_l_imu
    pause(1.5)
    viewer(quaternion(torso_imu.get_orientation.getAsMat', 'euler', 'XYZ', 'frame'));title('torso imu');% torso_imu
    pause(1.5)
    viewer(quaternion(pelvis_imu.get_orientation.getAsMat', 'euler', 'XYZ', 'frame'));title('pelvis imu');% pelvis_imu
    pause(1.5)
    viewer(quaternion(femur_r_imu.get_orientation.getAsMat', 'euler', 'XYZ', 'frame'));title('femur R imu');% femur_r_imu
    pause(1.5)
    viewer(quaternion(tibia_r_imu.get_orientation.getAsMat', 'euler', 'XYZ', 'frame'));title('tibia R imu');% tibia_r_imu
    pause(1.5)
    viewer(quaternion(calcn_r_imu.get_orientation.getAsMat', 'euler', 'XYZ', 'frame'));title('calcn R imu');% calcn_r_imu
    pause(1.5)
    viewer(quaternion(femur_l_imu.get_orientation.getAsMat', 'euler', 'XYZ', 'frame'));title('femur L imu');% femur_l_imu
    pause(1.5)
    viewer(quaternion(tibia_l_imu.get_orientation.getAsMat', 'euler', 'XYZ', 'frame'));title('tibia L imu');% tibia_l_imu
    pause(1.5)
    viewer(quaternion(calcn_l_imu.get_orientation.getAsMat', 'euler', 'XYZ', 'frame'));title('calcn L imu');% calcn_l_imu
end
%% COORD ESTIMATED correction
if coord_estimation
    q_Rpelvis = quaternion([Coord_Est.pelvis.list, Coord_Est.pelvis.tilt, Coord_Est.pelvis.rotation],'euler','XZY','frame');% OpenSim offset
    q_Rtorso = quaternion([-Coord_Est.lumbar.bending, -Coord_Est.lumbar.extension, -Coord_Est.lumbar.rotation],'euler','XZY','frame');% OpenSim offset
    q_Rfemr = quaternion([-Coord_Est.hip_r.adduction, -Coord_Est.hip_r.flexion, -Coord_Est.hip_r.rotation],'euler','XZY','frame');% OpenSim offset
    q_Rfeml = quaternion([-Coord_Est.hip_l.adduction, -Coord_Est.hip_l.flexion, -Coord_Est.hip_l.rotation],'euler','XZY','frame');% OpenSim offset
    q_Rtibr = quaternion([-Coord_Est.knee_r.adduction, -Coord_Est.knee_r.angle, -Coord_Est.knee_r.rotation],'euler','XZY','frame');% OpenSim offset
    q_Rtibl = quaternion([-Coord_Est.knee_l.adduction, -Coord_Est.knee_l.angle, -Coord_Est.knee_l.rotation],'euler','XZY','frame');% OpenSim offset
    q_Rcalr = quaternion([-Coord_Est.subtalar_r.angle, -Coord_Est.ankle_r.angle, 0],'euler','XZY','frame');% OpenSim offset
    q_Rcall = quaternion([-Coord_Est.subtalar_l.angle, -Coord_Est.ankle_l.angle, 0],'euler','XZY','frame');% OpenSim offset
end
%% Quaternion corrections
if ~coord_estimation
    q_Rhumr = quaternion([Coord.arm_r.adduction, Coord.arm_r.flexion, Coord.arm_r.rotation],'euler','XZY','frame');% OpenSim offset
    q_Rradr = quaternion([0, Coord.elbow_r.flexion, Coord.pro_sup_r.rotation],'euler','XZY','frame');% OpenSim offset
    q_Rhuml = quaternion([Coord.arm_l.adduction, Coord.arm_l.flexion, Coord.arm_l.rotation],'euler','XZY','frame');% OpenSim offset
    q_Rradl = quaternion([0, Coord.elbow_l.flexion, Coord.pro_sup_l.rotation],'euler','XZY','frame');% OpenSim offset
    q_Rpelvis = quaternion([Coord.pelvis.list, Coord.pelvis.tilt, Coord.pelvis.rotation],'euler','XZY','frame');% OpenSim offset
    q_Rtorso = quaternion([Coord.lumbar.bending, Coord.lumbar.extension, Coord.lumbar.rotation],'euler','XZY','frame');% OpenSim offset
    q_Rfemr = quaternion([Coord.hip_r.adduction, Coord.hip_r.flexion, Coord.hip_r.rotation],'euler','XZY','frame');% OpenSim offset
    q_Rfeml = quaternion([Coord.hip_l.adduction, Coord.hip_l.flexion, Coord.hip_l.rotation],'euler','XZY','frame');% OpenSim offset
    
%     q_Rfeml = quaternion([Coord.hip_l.adduction*0.5, Coord.hip_l.flexion*0.5, Coord.hip_l.rotation*0.5],'euler','XZY','frame');% OpenSim offset
    
    q_Rtibr = quaternion([Coord.knee_r.adduction, Coord.knee_r.angle, Coord.knee_r.rotation],'euler','XZY','frame');% OpenSim offset
    q_Rtibl = quaternion([Coord.knee_l.adduction, Coord.knee_l.angle, Coord.knee_l.rotation],'euler','XZY','frame');% OpenSim offset
    q_Rcalr = quaternion([0, Coord.ankle_r.angle, 0],'euler','XZY','frame');% OpenSim offset
    q_Rcall = quaternion([0, Coord.ankle_l.angle, 0],'euler','XZY','frame');% OpenSim offset
    
end


%% Offset correction
q_et = conj(q_Rpelvis)*conj(q_Rtorso)*q_calibrationM(1);% Torso 
q_ep = conj(q_Rpelvis)*q_calibrationM(2);% Pelvis

q_efr = conj(q_Rpelvis)*conj(q_Rfemr)*q_calibrationM(3);% Femur R
q_etr = conj(q_Rpelvis)*conj(q_Rfemr)*conj(q_Rtibr)*q_calibrationM(5);% Tibia R
q_ecar = conj(q_Rpelvis)*conj(q_Rfemr)*conj(q_Rtibr)*conj(q_Rcalr)*q_calibrationM(7);% Calcaneus R

q_efl = conj(q_Rpelvis)*conj(q_Rfeml)*q_calibrationM(4);% Femur L
q_etl = conj(q_Rpelvis)*conj(q_Rfeml)*conj(q_Rtibl)*q_calibrationM(6);% Tibia L
q_ecal = conj(q_Rpelvis)*conj(q_Rfeml)*conj(q_Rtibl)*conj(q_Rcall)*q_calibrationM(8);% Calcaneus L

q_ehr = conj(q_Rpelvis)*conj(q_Rtorso)*conj(q_Rhumr)*q_calibrationM(9);%  Humerus R
q_err = conj(q_Rpelvis)*conj(q_Rtorso)*conj(q_Rhumr)*conj(q_Rradr)*q_calibrationM(10);% Radius R
q_ehl = conj(q_Rpelvis)*conj(q_Rtorso)*conj(q_Rhuml)*q_calibrationM(11);% Humerus L
q_erl = conj(q_Rpelvis)*conj(q_Rtorso)*conj(q_Rhuml)*conj(q_Rradl)*q_calibrationM(12);% Radius L

%% VIEWER
if visualizeIMU
    viewer =   HelperOrientationViewer;

    viewer(q_et);title('Torso');% pelvis
    

    viewer(q_ep);title('Pelvis');% pelvis
    

    viewer(q_efr);title('Femur R');% femur R
    viewer(q_etr);title('Tibia R');% femur R
    viewer(q_ecar);title('Calcaneus R');% femur R
    viewer(q_efl);title('Femur L');% 
    viewer(q_calibrationM(4));title('Femur L');% 
    viewer(q_etl);title('Tibia L');% femur R
    viewer(q_ecal);title('CAlcaneus L');% femur R
    
    
end
%%

et = euler(q_et , 'XYZ', 'frame');% torso
ep = euler(q_ep , 'XYZ', 'frame');% pelvis
efr = euler(q_efr , 'XYZ', 'frame');% femur r 
etr = euler(q_etr , 'XYZ', 'frame');% tibia r
ecar = euler(q_ecar , 'XYZ', 'frame');% calcn r
efl = euler(q_efl , 'XYZ', 'frame');% femur l 
etl = euler(q_etl , 'XYZ', 'frame');% tibia l
ecal = euler(q_ecal , 'XYZ', 'frame');% calcn l
ehr = euler(q_ehr , 'XYZ', 'frame');% humerus r
err = euler(q_err , 'XYZ', 'frame');% radius r
ehl = euler(q_ehl , 'XYZ', 'frame');% humerus l
erl = euler(q_erl , 'XYZ', 'frame');% radius l

%% LOWERLIMBS ONLY
if LL_only
    ehr = 0.01*etr;
    err =  0.01*etr;
    ehl =  0.01*etr;
    erl =  0.01*etr;
end
%%
Or_Cal = [et; ep; efr; etr; ecar; efl; etl; ecal; ehr; err; ehl; erl];

%% Assign IMU orienation to the model
humerus_r_imu.set_orientation(Vec3(ehr(1), ehr(2), ehr(3)));
radius_r_imu.set_orientation(Vec3(err(1), err(2), err(3)));
humerus_l_imu.set_orientation(Vec3(ehl(1), ehl(2), ehl(3)));
radius_l_imu.set_orientation(Vec3(erl(1), erl(2), erl(3)));
torso_imu.set_orientation(Vec3(et(1), et(2), et(3)));
pelvis_imu.set_orientation(Vec3(ep(1), ep(2), ep(3)));
femur_r_imu.set_orientation(Vec3(efr(1), efr(2), efr(3)));
tibia_r_imu.set_orientation(Vec3(etr(1), etr(2), etr(3)));
calcn_r_imu.set_orientation(Vec3(ecar(1), ecar(2), ecar(3)));
femur_l_imu.set_orientation(Vec3(efl(1), efl(2), efl(3)));
tibia_l_imu.set_orientation(Vec3(etl(1), etl(2), etl(3)));
calcn_l_imu.set_orientation(Vec3(ecal(1), ecal(2), ecal(3)));

%% Assign IMU translation (just for visualize the sensors)
torso_imu.set_translation(Vec3(0.12, 0.32, 0));
pelvis_imu.set_translation(Vec3(-0.15, 0.08, 0)) ;
femur_r_imu.set_translation(Vec3(0.05, -0.15, 0.08));
tibia_r_imu.set_translation(Vec3(0.05, -0.15, -0.05));
calcn_r_imu.set_translation(Vec3(0.15, 0.05, 0.02));
femur_l_imu.set_translation(Vec3(0.05, -0.15, -0.08));
tibia_l_imu.set_translation(Vec3(0.05, -0.15, 0.05));
calcn_l_imu.set_translation(Vec3(0.15, 0.05, -0.02));
humerus_r_imu.set_translation(Vec3(0.04, -0.15, 0.03));
radius_r_imu.set_translation(Vec3(-0.04, -0.18, 0.01));
humerus_l_imu.set_translation(Vec3(0.04, -0.15, -0.03));
radius_l_imu.set_translation(Vec3(-0.04, -0.18, -0.01));

%% SAVE Or_Cal matrix
if s < 10
    save(['Or_Cal_S0', num2str(s),'_P', num2str(p),'.mat'], 'Or_Cal', 'align_anglesM', 'q_calibration');
else
    save(['Or_Cal_S', num2str(s),'_P', num2str(p), '.mat'], 'Or_Cal', 'align_anglesM', 'q_calibration');
end
disp('Or_Cal saved')

%% SAVE model Calibrated
if s<10
    modelName = [ 'S0', num2str(s),'_allMarkers_allIMU'];
else
    modelName = [ 'S', num2str(s),'_allMarkers_allIMU'];
end
if p > 1
    new_model_Name = [modelName, '_CoordEst_calibrated.osim'];
else
    new_model_Name = [modelName, '_calibrated.osim'];
end
model.print(new_model_Name);
disp('Model Saved')

pause(2)
disp([char(13), '-------- Calibration done ----------'])