function result = split_and_merge(points,epsilon)
% Find the point with maximum distance
dmax = 0;
end_point = length(points);
for i = 2:end_point-1
    d = penDistance(points(:,i),points(:,1),points(:,end_point));
    if d > dmax
        index = i;
        dmax = d;
    end
end
% If max distance is greater than epsilon, recursively get the result
if dmax > epsilon
    % recursive call
    recResult1 = split_and_merge(points(:,1:index),epsilon);
    recResult2 = split_and_merge(points(:,index:end_point),epsilon);
    result = [recResult1(:,1:length(recResult1)-1) recResult2(:,1:length(recResult2))];
else
    result = [points(:,1) points(:,end_point)];
end
    function d = penDistance(pmain, p1, p2)
        % distance between pmain and the line made by p1 and p2
        % A*X + B*Y + C = 0   A = y1 - y2   B = x2 - x1   C = x1*y2 - x2*y1
        % distance function =>  |Ax1 + By1 + C|/sqrt(A^2 + B^2)
        d = abs((p2(2,1)-p1(2,1))*pmain(1,1) - (p2(1,1)-p1(1,1))*pmain(2,1) + p2(1,1)*p1(2,1) - p2(2,1)*p1(1,1)) ...
            / sqrt((p2(2,1)-p1(2,1))^2 + (p2(1,1)-p1(1,1))^2);
    end
end