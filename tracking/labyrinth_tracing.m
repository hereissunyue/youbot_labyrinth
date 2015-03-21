%% Preprocessing
addpath('C:\Users\sunyue\Desktop\Project') % path of video
addpath('C:\Users\sunyue\Desktop\Project\youbot_labyrinth\tracking\library')  % path of library
addpath('C:\Users\sunyue\Desktop\Project\youbot_labyrinth\image')  % path of labyrinth reference library

video1 = VideoReader('1.avi'); % read the video 1
video2 = VideoReader('2.avi'); % read the video 2
frame2 = video2.NumberOfFrames; % get the frame number of the video 2
frame1 = frame2/2; % making the 


%% Extract the Map of the labyrinth
Map = zeros(310,310); % plack map first
RGBimg = read(video1,65);% image of Red Channel
RGBimg = imcrop(RGBimg,[83 5 578 568]); % Extract the map region
RGBimg = imresize(RGBimg, [310 310]); % resize the map to be specific size
HSVimg = rgb2hsv(RGBimg); % transfer the image into HSV form
Boundary = ones(310,310);  % boundary of the labyrinth
for i = 1:310
    for j = 1:310
        if i <=10 || i>=300 || j<=10 || j>=300
           Boundary(i,j) = 0;
        end
    end
end


%% Locate the start region
start = HSVfilter(RGBimg, [0.3433 0.5034 0.2717], [0.03 0.1 0.1]); % extract the start region of the labyrinth
se90 = strel('line', 3, 90);
se0 = strel('line', 3, 0);
start = imdilate(start, [se90 se0]); % Dilation make line more noticable
start = bwareaopen(start,20);  % Eliminate small region
start_region = [];
% calculating the region status
for i = 1 : 31
    for j = 1 : 31
        sample = start(10*(i-1)+1:10*i,10*(j-1)+1:10*j); % temp sample
        check = mean(mean(sample)); % calculate the average value in that region
        if check > 0.5
            start_region = [start_region; 10*(j-1)+1,10*(i-1)+1,10,10]; %#ok<AGROW>
        end
    end
end
start_region_num = size(start_region,1);
start_mask = zeros(310,310); % initial start_mask
for k = 1:start_region_num
    start_mask(start_region(k,2):start_region(k,2)+9,start_region(k,1):start_region(k,1)+9) = 1;
end


%% Locate the goal region
goal = HSVfilter(RGBimg, [0.58 0.7635 0.38], [0.05 0.2 0.2]); % extract the start region of the labyrinth
se90 = strel('line', 3, 90);
se0 = strel('line', 3, 0);
goal = imdilate(goal, [se90 se0]); % Dilation make line more noticable
goal = Boundary & goal;  % remove boundary distraction
goal = bwareaopen(goal,100);  % Eliminate small region
goal_region = [];
% calculating the region status
for i = 1 : 31
    for j = 1 : 31
        sample = goal(10*(i-1)+1:10*i,10*(j-1)+1:10*j); % temp sample
        check = mean(mean(sample)); % calculate the average value in that region
        if check > 0.5
            goal_region = [goal_region; 10*(j-1)+1,10*(i-1)+1,10,10]; %#ok<AGROW>
        end
    end
end
goal_region_num = size(goal_region,1);
goal_mask = zeros(310,310); % initial start_mask
for k = 1:goal_region_num
    goal_mask(goal_region(k,2):goal_region(k,2)+9,goal_region(k,1):goal_region(k,1)+9) = 1;
end


%% Locate the wall region
reference = imread('laybrinth.png');   % set up labyrinth reference
reference1 = im2bw(reference, 0.8);
reference2 = imrotate(reference1,90);
reference3 = imrotate(reference1,180);
reference4 = imrotate(reference1,270);

% extract rough wall from the image
wall = im2bw(RGBimg, 0.5);  % binarize the RGB image
wall = wall & Boundary | (goal_mask | start_mask);
wall_region = [];
% calculating the region status
for i = 1 : 31
    for j = 1 : 31
        sample = wall(10*(i-1)+1:10*i,10*(j-1)+1:10*j); % temp sample
        check = mean(mean(sample)); % calculate the average value in that region
        if check > 0.4
            wall_region = [wall_region; 10*(j-1)+1,10*(i-1)+1,10,10]; %#ok<AGROW>
        end
    end
end
wall_region_num = size(wall_region,1);
wall_mask = zeros(310,310); % initial start_mask
for k = 1:wall_region_num
    wall_mask(wall_region(k,2):wall_region(k,2)+9,wall_region(k,1):wall_region(k,1)+9) = 1;
end

% calculate the wall reference difference
check1 = sum(sum(abs(reference1 - wall_mask)));
check2 = sum(sum(abs(reference2 - wall_mask)));
check3 = sum(sum(abs(reference3 - wall_mask)));
check4 = sum(sum(abs(reference4 - wall_mask)));

% choose the right wall reference
if check1<15000
    wall = reference1;
elseif check2<15000
    wall = reference2;
elseif check2<15000
    wall = reference3;
elseif check2<15000
    wall = reference4;
end

wall_region = [];
% calculating the region status
for i = 1 : 31
    for j = 1 : 31
        sample = wall(10*(i-1)+1:10*i,10*(j-1)+1:10*j); % temp sample
        check = mean(mean(sample)); % calculate the average value in that region
        if check > 0.4
            wall_region = [wall_region; 10*(j-1)+1,10*(i-1)+1,10,10]; %#ok<AGROW>
        end
    end
end
wall_region_num = size(wall_region,1);
wall_mask = zeros(310,310); % initial start_mask
for k = 1:wall_region_num
    wall_mask(wall_region(k,2):wall_region(k,2)+9,wall_region(k,1):wall_region(k,1)+9) = 1;
end


%% Extract the Path Node
path_node = [];
% calculating the region status
for i = 1 : 31
    for j = 1 : 31
        sample = wall_mask(10*(i-1)+1:10*i,10*(j-1)+1:10*j); % temp sample
        check = mean(mean(sample)); % calculate the average value in that region
        if check > 0.8
            path_node = [path_node; 10*(j-1) + 5.5,10*(i-1) + 5.5]; %#ok<AGROW>
        end
    end
end
path_node_num = size(path_node,1);



%% Draw the Map

% figure(1)
% imshow(Map)
% hold on
% for k = 1:wall_region_num
%     rectangle('Position',wall_region(k,:),'FaceColor','w','EdgeColor','w')
%     hold on
% end
% for k = 1:start_region_num
%     rectangle('Position',start_region(k,:),'FaceColor','g','EdgeColor','g')
%     hold on
% end
% for k = 1:goal_region_num
%     rectangle('Position',goal_region(k,:),'FaceColor','b','EdgeColor','b')
%     hold on
% end
% plot(path_node(:,1),path_node(:,2),'r*')
% hold off



