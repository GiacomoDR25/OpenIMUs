% quat_mean Computes the average quaternion
%    qMean = quat_mean(qs) computes the average quaternion qMean of the
%    N quaternions stored in the N-by-4 matrix qs.
%    Reference: http://www.acsu.buffalo.edu/~johnc/ave_quat07.pdf 
%               F. Landis Markley: Averaging Quaternions
function qMean = quat_mean(qs)
    
    % Convert qs into form q = [axis' scalar]' where axis is a column
    % vector
    qsMod = qs';
    qsMod = [qsMod(2:4,:); qsMod(1,:)];

    M = zeros(4,4);
    for i=1:size(qsMod,2)
        M = M+qsMod(:,i)*qsMod(:,i)';
    end
    
    [vectors, values] = eig(M);    
    [~, inSorted] = sort(diag(values));
    
    qMean = vectors(:,inSorted(end))';
    qMean = [qMean(4) qMean(1:3)];
end