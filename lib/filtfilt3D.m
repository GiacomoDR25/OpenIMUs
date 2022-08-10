% filtfilt3D Exactly same function as filtfilt but works on 3D data
%     filtered = filtfilt3D(b,a,rawData) applies the function filtfilt
%     separately on each of the three columns of rawData. 

function filtered = filtfilt3D(b,a,rawData)
    filtered(:,1) = filtfilt(b,a,rawData(:,1));
    filtered(:,2) = filtfilt(b,a,rawData(:,2));
    filtered(:,3) = filtfilt(b,a,rawData(:,3));
end
