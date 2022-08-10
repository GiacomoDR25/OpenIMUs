% convertFrames Converts IMU data into a difference frame
%     globalFrame = convertFrames(imuData, segmentOrientation) converts the
%     inertial data imuData (structure with fields acc and gyr) into the
%     new frame given by the time-dependent transform segmentOrientation(a
%     list of quaternions). globalFrame has the same structure as imuData
%     but with the inertial data expressed in the new frame

function globalFrame = convertFrames(imuData, segmentOrientation)


    globalFrame.acc = zeros(size(imuData.acc));
    globalFrame.gyr = zeros(size(imuData.gyr));
    
    if isfield(imuData, 'mag')
        globalFrame.mag = zeros(size(imuData.mag));

        for i=1:size(segmentOrientation,1)-1
            segmentR = quat2matrix(segmentOrientation(i,:))';

            globalFrame.acc(i,:) = imuData.acc(i,:)*segmentR;
            globalFrame.gyr(i,:) = imuData.gyr(i,:)*segmentR;
            globalFrame.mag(i,:) = imuData.mag(i,:)*segmentR;
        end
        
    else
    
        for i=1:size(segmentOrientation,1)-1
            segmentR = quat2matrix(segmentOrientation(i,:))';

            globalFrame.acc(i,:) = imuData.acc(i,:)*segmentR;
            globalFrame.gyr(i,:) = imuData.gyr(i,:)*segmentR;
        end
    end
end