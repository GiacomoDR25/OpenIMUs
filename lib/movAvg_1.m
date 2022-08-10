% movAvg Computes centered moving average and standard deviation
%     [movMean, movStd] = movAvg(sequence, windowsize) returns the centered moving
%     average and standard deviation for sequence and average length
%     windowsize

function [movMean, movStd] = movAvg(sequence, windowsize)
    sequenceLength = numel(sequence);
    
    movMean = zeros(sequenceLength-windowsize, 1);
    movVar = zeros(sequenceLength-windowsize, 1);
    
    % Compute average and sd for the first block of data
    movMean(1) = mean(sequence(1:windowsize));
    movVar(1) = var(sequence(1:windowsize));
    
    % Compute moving average and std according to 
    % http://stackoverflow.com/questions/14635735/how-to-efficiently-calculate-a-moving-standard-deviation
    for i=2:sequenceLength-windowsize
        diffFirstLast = sequence(windowsize+i-1)-sequence(i-1);
        movMean(i) = movMean(i-1) + diffFirstLast/windowsize;
        movVar(i) = movVar(i-1) + (sequence(windowsize+i-1)-movMean(i) + sequence(i-1)-movMean(i-1))*diffFirstLast/(windowsize-1);
    end
    
    padding = zeros(ceil(windowsize/2), 1);
    movMean = [padding; movMean; padding];
    movStd = [padding; sqrt(movVar); padding];
    
    if mod(windowsize,2)==1
        movMean = movMean(1:end-1);
        movStd = movStd(1:end-1);
    end
end