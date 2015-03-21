%% Preprocessing
addpath('C:\Users\sunyue\Desktop\Project') % path of video
addpath('C:\Users\sunyue\Desktop\Project\youbot_labyrinth\tracking\library')  % path of library
addpath('C:\Users\sunyue\Desktop\Project\youbot_labyrinth\image')  % path of labyrinth reference library

video1 = VideoReader('1.avi'); % read the video 1
video2 = VideoReader('2.avi'); % read the video 2
frame2 = video2.NumberOfFrames; % get the frame number of the video 2
frame1 = video1.NumberOfFrames; % making the frame syncronize


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
start_node = [];
% calculating the region status
for i = 1 : 31
    for j = 1 : 31
        sample = start(10*(i-1)+1:10*i,10*(j-1)+1:10*j); % temp sample
        check = mean(mean(sample)); % calculate the average value in that region
        if check > 0.5
            start_region = [start_region; 10*(j-1)+1,10*(i-1)+1,10,10]; %#ok<AGROW>
            start_node = [start_node; j,i]; %#ok<AGROW>
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
goal_node = [];
% calculating the region status
for i = 1 : 31
    for j = 1 : 31
        sample = goal(10*(i-1)+1:10*i,10*(j-1)+1:10*j); % temp sample
        check = mean(mean(sample)); % calculate the average value in that region
        if check > 0.5
            goal_region = [goal_region; 10*(j-1)+1,10*(i-1)+1,10,10]; %#ok<AGROW>
            goal_node = [goal_node; j,i]; %#ok<AGROW>
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
obstacle = [];
% calculating the region status
for i = 1 : 31
    for j = 1 : 31
        sample = wall(10*(i-1)+1:10*i,10*(j-1)+1:10*j); % temp sample
        check = mean(mean(sample)); % calculate the average value in that region
        if check > 0.4
            wall_region = [wall_region; 10*(j-1)+1,10*(i-1)+1,10,10]; %#ok<AGROW>
        else
            obstacle = [obstacle; j,i]; %#ok<AGROW>
        end
    end
end
wall_region_num = size(wall_region,1);
wall_mask = zeros(310,310); % initial start_mask
for k = 1:wall_region_num
    wall_mask(wall_region(k,2):wall_region(k,2)+9,wall_region(k,1):wall_region(k,1)+9) = 1;
end


%% Extract the Path Node
path_location = [];
path_node = [];
% calculating the region status
for i = 1 : 31
    for j = 1 : 31
        sample = wall_mask(10*(i-1)+1:10*i,10*(j-1)+1:10*j); % temp sample
        check = mean(mean(sample)); % calculate the average value in that region
        if check > 0.8
            path_location = [path_location; 10*(j-1) + 5.5,10*(i-1) + 5.5]; %#ok<AGROW>
            path_node = [path_node;j,i]; %#ok<AGROW>
        end
    end
end
path_node_num = size(path_location,1);


%% path planning
start = start_node(1,:);
goal = goal_node(2,:);
[pathlist] = Astar(start, goal, obstacle);  % implement A star to find the path
real_path_location = 10*(pathlist - 1) + 5.5;

% extract the corner of the path
[cornerpath] = pathcorner(pathlist);
corner_path_location = 10*(cornerpath - 1) + 5.5;


%% Draw the Map

figure(100)
imshow(Map)
hold on
for k = 1:wall_region_num
    rectangle('Position',wall_region(k,:),'FaceColor','w','EdgeColor','w')
    hold on
end
for k = 1:start_region_num
    rectangle('Position',start_region(k,:),'FaceColor','g','EdgeColor','g')
    hold on
end
for k = 1:goal_region_num
    rectangle('Position',goal_region(k,:),'FaceColor','b','EdgeColor','b')
    hold on
end
% plot(corner_path_location(:,1),corner_path_location(:,2),'m','Linewidth',3)
frame = getframe;
map = frame.cdata;
close all


%% Tracking Preparation
RGBimg = read(video1,180);% image of Red Channel
RGBimg = imcrop(RGBimg,[83 5 578 568]); % Extract the map region
RGBimg = imresize(RGBimg, [310 310]); % resize the map to be specific size
ball = HSVfilter(RGBimg, [0.056 0.5117 0.78], [0.1 0.3 0.3]); % extract the start region of the labyrinth
ball = bwareaopen(ball,20);  % Eliminate small region
se90 = strel('line', 3, 90);
se0 = strel('line', 3, 0);
ball = imdilate(ball, [se90 se0]); % Dilation make line more noticable
ball = bwareaopen(ball,40);  % Eliminate small region
ball_info = regionprops(ball); % Getting connected region information
centroid = cat(1, ball_info.Centroid); % Getting centroid
% [centroid] = onpath(centroid);
% 
% imshow(map)
% hold on
% viscircles(centroid,2,'EdgeColor','r','Linewidth',6);
% hold off


%% Kalman Filter Implementation on Brazil Players Tracking
dT = 1/30; % set T to be 33 based on ground truth
At_vel = [1,0,dT,0;0,1,0,dT;0,0,1,0;0,0,0,1]; % At in velocity constant model
Ct_vel = [1,0,0,0;0,1,0,0]; % Ct in velocity constant model
acceleration_noise = 100; % the variability in how fast the player is speeding up (stdv of acceleration: meters/sec^2)
measurement_noise_x = 0.01;  %measurement noise in the horizontal direction (x axis).
measurement_noise_y = .000001;  %measurement noise in the horizontal direction (y axis).
Qt_vel = [measurement_noise_x 0; 0 measurement_noise_y]; % Qt in velocity constant model
Rt_vel = [dT^4/4 0 dT^3/2 0; 0 dT^4/4 0 dT^3/2; dT^3/2 0 dT^2 0; 0 dT^3/2 0 dT^2].*acceleration_noise^2; % Ct in velocity constant model

% extracting the measurement data
state_X = centroid_B(:,1);  % X values
state_Y = centroid_B(:,2);  % Y values
state_space = [state_X, state_Y, zeros(size(centroid_B,1),1), zeros(size(centroid_B,1),1)]';  % Set up state vector
state_vector = nan(4,11);  % save enough space for more detection tracking but maximum 11 players so set 11
state_vector(:,1:size(state_space,2)) = state_space;  %estimate of initial location estimation of where the flies are(what we are updating)
position_estimateX = nan(11); %  position estimate
position_estimateY = nan(11); %  position estimate

% initial value
Et_1 = Rt_vel; % The starting covariance
nF = find(isnan(state_vector(1,:))==1,1)-1 ; %initize number of track estimates

strk_trks = zeros(1,11);  %counter of how many strikes a track has gotten


%% Tracking video
writerObj = VideoWriter('ball.avi');
open(writerObj);
last = corner_path_location(1,:);
trajectory = [];
for k = 1:frame1
    
        current_state = centroid_B;
        nD = size(centroid_B,1); %set new number of detections
        
       %% implementation of Kalman Filter half
        for i = 1:nF
            state_vector(:,i) = At_vel * state_vector(:,i); % Prediction of State Vector resulting Xtba
        end
        Etba = At_vel * Et_1 * transpose(At_vel) + Rt_vel; % Prediction of State Covariance
        Kt =  Etba * transpose(Ct_vel) / (Ct_vel * Etba * transpose(Ct_vel) + Qt_vel);  % Kalman Gain

       %% Hungarian algorithm implementation
        % make the distance (cost) matrice between all pairs rows are tracks, column are detections
        distance = pdist([state_vector(1:2,1:nF)'; current_state]); % calculate the distance
        distance = squareform(distance); %make square
        distance = distance(1:nF,nF+1:end) ; %limit to just the tracks to detection distances
        % Doing assignment optimal calculation based on Munkres algorithm
        [assignment, cost] = AssignmentOptimal(distance);
        assignment = assignment'; % correct the assignment output in appropriate form
        
        % check for tough situations and if it's tough, ignore the data make assignment = 0 for that tracking element 
        % check whether the detection far from the observation
        rejection = [];
        for j = 1:nF
            if assignment(j) > 0
                rejection(j) =  distance(j,assignment(j)) < 50 ; %#ok<SAGROW>
            else
                rejection(j) = 0; %#ok<SAGROW>
            end
        end
        assignment = assignment.*rejection;
         
       %% implementation of Kalman Filter Second half
        m = 1; 
        for j = 1:length(assignment)
            if assignment(j) > 0
                state_vector(:,m) = state_vector(:,m) + Kt * (current_state(assignment(j),:)' - Ct_vel * state_vector(:,m));
            end
            m = m + 1;
        end
        I = eye(size(Et_1));
        Et = (I - Kt * Ct_vel) * Etba; % Correction of State Covariance
        centroid_B = state_vector(1:2,:)';  % Updata the centroid_B value

      %% Update new detection
       %everything that doesn't get assigned is a new tracking
       new_detection = current_state(~ismember(1:size(current_state,1),assignment),:)';
       if ~isempty(new_detection)
           state_vector(:,nF+1:nF+size(new_detection,2))=  [new_detection; zeros(2,size(new_detection,2))];
           nF = nF + size(new_detection, 2);  % number of track estimates with new ones included
       end 
       
      %% Update missing detetion
       %give a strike to any tracking that didn't get matched up to a detection
       miss_tracking =  find(assignment==0);
       if ~isempty(miss_tracking)
           strk_trks(miss_tracking) = strk_trks(miss_tracking) + 1;
       end
       bad_tracking = find(strk_trks > 11); %if a track has a strike greater than 6, delete the tracking
       state_vector(:,bad_tracking) = NaN;       
        % k = 1 do noting
    oldcentroid = centroid_B;
    num = size(centroid_B,1); %set new number of detections
    
    figure(100)
    imshow(map)
    hold on
    a = centroid(2);
    viscircles([centroid(1),a],2,'EdgeColor','r','Linewidth',6);
    hold off
    frame = getframe;
    writeVideo(writerObj,frame);
    
    trajectory = [trajectory;centroid];  %#ok<AGROW>
end
close(writerObj);