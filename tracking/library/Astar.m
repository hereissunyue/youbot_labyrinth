% [pathlist] = Astar(current, goal, path_node, obstacle)
function [pathlist] = Astar(start, goal, obstacle)
pathlist = []; % paht list
current = start; % current node is start node
last = [0,0]; % last node
backup = []; % back up node in case encounter an dead end

while sum(current==goal)~=2
    north = current + [0,1];  % north option
    south = current + [0,-1];  % south option
    east = current + [1,0];  % east option
    west = current + [-1,0];   % west option
    
    % heuristic cost of north node
    if ismember(north,obstacle,'rows')==0 && (sum(north==last)~=2) 
        % north is not in the obstacle list and it is not the last node
        Ncost = 1 + norm(abs(north-goal));
    else
        Ncost = 10000 + norm(abs(north-goal)); % otherwise the cost is very big
    end 

    % heuristic cost of south node
    if ismember(south,obstacle,'rows')==0 && (sum(south==last)~=2) 
        % north is not in the obstacle list and it is not the last node
        Scost = 1 + norm(abs(south-goal));
    else
        Scost = 10000 + norm(abs(south-goal)); % otherwise the cost is very big
    end 

    % heuristic cost of esat node
    if ismember(east,obstacle,'rows')==0 && (sum(east==last)~=2) 
        % north is not in the obstacle list and it is not the last node
        Ecost = 1 + norm(abs(east-goal));
    else
        Ecost = 10000 + norm(abs(east-goal)); % otherwise the cost is very big
    end 

    % heuristic cost of esat node
    if ismember(west,obstacle,'rows')==0 && (sum(west==last)~=2) 
        % north is not in the obstacle list and it is not the last node
        Wcost = 1 + norm(abs(west-goal));
    else
        Wcost = 10000 + norm(abs(west-goal)); % otherwise the cost is very big
    end 
    
    next_option = [north, Ncost; south, Scost; east, Ecost; west, Wcost];
    next_option = sortrows(next_option,3);  % set up a cost matrix and sort based on cost
    
    if next_option(1,3)<100 && next_option(2,3)> 1000  % there is only one next step
        last = current;  % update the last node
        current = next_option(1,1:2);  % update the next node
        pathlist = [pathlist;current]; %#ok<AGROW>  % store the node in the path list
        
    elseif  next_option(1,3)<100 && next_option(2,3)<100 % there are two options
        backup = [next_option(2,1:2), current; backup]; %#ok<AGROW> % backup in case dead end
        last = current;  % update the last node
        current = next_option(1,1:2);  % update the next node
        pathlist = [pathlist;current]; %#ok<AGROW>  % store the node in the path list
        
    elseif next_option(1,3)>1000 % there is no next option
        current = backup(1,1:2);  % current is backup number 1
        last = backup(1,3:4);  % last node is back up first line second part
        backup = backup(2:end,:); % remove the first line because the back up is activated
        % remove everything till last
        num = size(pathlist,1); % go through all the path list until find it
        for i = 1:num
            if sum(pathlist(i,:)==last)==2 % find the last
                index = i;
                break % break the for loop
            end
        end
        pathlist = pathlist(1:index,:); % remove all path nodes after the last node
        pathlist = [pathlist;current]; %#ok<AGROW>  % store the node in the path list
    end
end

pathlist = [start;pathlist;goal]; % add start and goal to tha pathlist
end