%% Preprocessing
addpath('C:\Users\sunyue\Desktop\Project') % path of video
addpath('C:\Users\sunyue\Desktop\Project\youbot_labyrinth\tracking\library')  % path of library
addpath('C:\Users\sunyue\Desktop\Project\youbot_labyrinth\image')  % path of labyrinth reference library

video1 = VideoReader('1.avi'); % read the video 1
video2 = VideoReader('2.avi'); % read the video 2
video3 = VideoReader('ball.avi'); % read the video 1
frame2 = video2.NumberOfFrames; % get the frame number of the video 2
frame1 = video1.NumberOfFrames; % making the frame syncronize
frame3 = video3.NumberOfFrames; % making the frame syncronize

%% Extract the Map of the labyrinth
img1 = read(video1,1);% image of Red Channel
img1 = imcrop(img1,[83 5 578 568]); % Extract the map region
img1 = imresize(img1, [320 320]); % resize the map to be specific size
img2 = read(video2,1);% image of Red Channel
img2 = imresize(img2, [320 181]); % resize the map to be specific size
img3 = read(video3,1);
img3 = imresize(img3, [320 320]); % resize the map to be specific size
blank = zeros(320,20,3);
up = zeros(20,901,3);




writerObj = VideoWriter('ball.avi');
open(writerObj);



for i = 1 : frame1
    if 2*i<=frame2 
        img1 = read(video1,i);% image of Red Channel
        img1 = imcrop(img1,[83 5 578 568]); % Extract the map region
        img1 = imresize(img1, [320 320]); % resize the map to be specific size
        img2 = read(video2,2*i);% image of Red Channel
        img2 = imresize(img2, [320 181]); % resize the map to be specific size
        img3 = read(video3,i);
        img3 = imresize(img3, [320 320]); % resize the map to be specific size
        pintu = [up;blank,img2, blank, img1, blank, img3, blank;up];
    elseif 2*i>frame2
        img1 = read(video1,i);% image of Red Channel
        img1 = imcrop(img1,[83 5 578 568]); % Extract the map region
        img1 = imresize(img1, [320 320]); % resize the map to be specific size
        img2 = read(video2,end);% image of Red Channel
        img2 = imresize(img2, [320 181]); % resize the map to be specific size
        img3 = read(video3,i);
        img3 = imresize(img3, [320 320]); % resize the map to be specific size
        pintu = [up;blank,img2, blank, img1, blank, img3, blank;up];
    end
    figure(1)
    imshow(pintu)
    frame = getframe;
    writeVideo(writerObj,frame);
end
close(writerObj);



