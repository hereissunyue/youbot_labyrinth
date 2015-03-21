% Put the detection of the ball on the path grid
function [centroid] = onpath(centroid, path_location)
num = size(path_location,1);
cost = [];
for i = 1: num
    temp = path_location(i,:);
    error = sum(abs(centroid - temp));
    cost = [cost;error, i]; %#ok<AGROW>
end
cost = sortrows(cost,1);
index = cost(1,2);
centroid = path_location(index,:);   
end