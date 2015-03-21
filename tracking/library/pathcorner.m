% Given a list of path and extract the corner node out
function [path] = pathcorner(pathlist)
num = size(pathlist,1); % total path nodes
last = pathlist(1,:); % the first node of the path
path = last; % the first is start goal
for i = 2 : num
    next = pathlist(i,:);
    if sum(next == last) == 0 % totally different
        path = [path; pathlist(i-1,:)];  %#ok<AGROW> % store path
        last = pathlist(i-1,:); % update next
    end     
end
path = [path;pathlist(end,:)]; % add the last node as the goal
end