% translateAcc Translates acceleration to a virtual point
%     acc_translated = translateAcc(imuData, translation, gravityValue, fs)
%     translates the acceleration measured in imuData by translation meters
%     relative to the sensor position (i.e. from the physical sensor position).
%     The translation is performed in the local frame in which imuData is
%     represented.
%     Returned is the acceleration at the new position, in g
%     acc_translated = translateAcc(imuData, translation, gravityValue, fs, unitsInG)
%     alternatively allows to directly use m/s2 as units for the
%     acceleration if unitsInG is false. In this case returned acceleration 
%     is in m/s^2. By default unitsInG is true and returned are g. 

function acc_translated = translateAcc(imuData, translation, gravityValue, fs, unitsInG)

    if nargin<5
        unitsInG = true;
    end
    
    % Convert into rad/sec   
    imuAngVel = imuData.gyr ./ 180 * pi;
    
    % Compute the angular acceleration and retrieve the acceleration
    imuAngAcc = [0 0 0; (imuAngVel(3:end,:)-imuAngVel(1:end-2,:)).*(fs/2); 0 0 0];
    
    % Lowpass filter slightly
    [b,a] = butter(2, 1.2465*20/fs*2, 'low');
    imuAngAcc = filtfilt3D(b,a,imuAngAcc);
    
    % Translate acceleration from g to m/2sec2
    if unitsInG
        imuAcc = imuData.acc*gravityValue;
    else
        imuAcc = imuData.acc;
    end

    % Translate to new point
    analysisLength = size(imuData.acc,1);
    acc_translated = zeros(analysisLength,3);

    for i=1:analysisLength
        % Formula:
        % acc_new = acc_old + cross(angAcc, r) + cross(angVel, cross(angVel, r)) 
        
        % compute cross(angAcc(i,:), r)
        a = imuAngAcc(i,:); b = translation;
        c1 = [a(2).*b(3)-a(3).*b(2) a(3).*b(1)-a(1).*b(3) a(1).*b(2)-a(2).*b(1)];
    
        % Compute cross(angVel(i,:), r)
        a = imuAngVel(i,:); b = translation;
        c2 = [a(2).*b(3)-a(3).*b(2) a(3).*b(1)-a(1).*b(3) a(1).*b(2)-a(2).*b(1)];
        
        % Compute cross(wThigh(i,:), c2)
        a = imuAngVel(i,:); b = c2;
        c3 = [a(2).*b(3)-a(3).*b(2) a(3).*b(1)-a(1).*b(3) a(1).*b(2)-a(2).*b(1)];
    
        acc_translated(i,:) = imuAcc(i,:) + c1 + c3;
        
        % Old formula: slow because of use of function cross
        % acc_translated(i,:) = imuAcc(i,:) + cross(imuAngAcc(i,:), translation) + cross(imuAngVel(i,:), cross(imuAngVel(i,:), translation));
    end
   
    if unitsInG
        acc_translated = acc_translated./gravityValue;
    end
end