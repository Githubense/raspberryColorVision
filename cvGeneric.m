clc                                                                         % Clear command window
clear all;                                                                  % Clear all workspace variables
close all;                                                                  % Close all open figures

%% Definition of robot
a1=5;                                                                       % Length of link 1
a2=3;                                                                       % Length of link 2
a3=3;                                                                       % Length of link 3
a4=5;                                                                       % Length of link 4

% Link Definition
H1 = Link([0,a1,0,3*pi/2,1]);                                               % Create link 1 object with parameters: theta=0, d=a1, a=0, alpha=-pi/2
H1.qlim = [0 0];                                                            % Set joint limits for link 1
H2 = Link([3*pi/2,a2,0,3*pi/2,1]);                                          % Create link 2 object with parameters: theta=-pi/2, d=a2, a=0, alpha=-pi/2
H2.qlim = [0 5];                                                            % Set joint limits for link 2
H3 = Link([3*pi/2,a3,0,3*pi/2,1]);                                          % Create link 3 object with parameters: theta=-pi/2, d=a3, a=0, alpha=-pi/2
H3.qlim = [0 5];                                                            % Set joint limits for link 3
H4 = Link([0,a4,0,0,1]);                                                    % Create link 4 object with parameters: theta=0, d=a4, a=0, alpha=0
H4.qlim = [0 5];                                                            % Set joint limits for link 4

% Link aggregation
r3bot=SerialLink([H1 H2 H3 H4],'name','robot');                             % Create a 4-link robotic arm object

% Robot plotting
r3bot.plot([0 0 0 0], 'workspace', [-50 50 -50 50 -50 50]);                 % Plot the robotic arm in its initial position

%% Image Acquisition Config

a = imaqhwinfo;                                                             % Get information about the available image acquisition devices

f1=1;                                                                       % Figure 1 handle
f2=2;                                                                       % Figure 2 handle

objects=imaqfind;                                                           % Find all image acquisition objects currently in memory
delete(objects);                                                            % Delete all image acquisition objects currently in memory

imaqmex('feature','-limitPhysicalMemoryUsage',false);                       % Configure memory usage for image acquisition

vid = videoinput('winvideo',3);                                             % Create a video input object

set(vid, 'FramesPerTrigger', Inf);                                          % Set the number of frames to acquire to Inf
set(vid, 'ReturnedColorspace', 'rgb')                                       % Set the returned color space to RGB
vid.FrameGrabInterval = 1;                                                  % Set the frame grab interval to 1

start(vid)                                                                  % Start the video input object

%% Image Processing
while(1)                                                                    % Run an infinite loop to capture and process images

    data = getsnapshot(vid);                                                % Capture a snapshot from the video input object
    diff_im = imsubtract(data(:,:,1), rgb2gray(data));                      % Subtract the red channel from the grayscale image of the snapshot
    diff_im = medfilt2(diff_im, [3 3]);                                     % Apply a 3x3 median filter to remove noise
    diff_im = im2bw(diff_im,0.18);                                          % Convert the filtered image to binary format using a threshold value of 0.18.
    diff_im = bwareaopen(diff_im,300);                                      % Remove objects in the binary image with fewer than 300 pixels.
    bw = bwlabel(diff_im, 8);                                               % Label the connected components in the binary image using 8-connectivity.
    
    stats = regionprops(bw, 'BoundingBox', 'Centroid');                     % Extract properties of the labeled regions, such as the bounding box and centroid.
    
    figure(f1)                                                              % Create a new figure window
    imshow(data)                                                            % Display the original captured image on the figure window.

%% Positioning
    for object = 1:length(stats)                                            % Loop through each labeled object in the image.
    
        bb = stats(object).BoundingBox;                                     % Extract the bounding box coordinates of the current object.
    
        bc = stats(object).Centroid;                                        % Extract the centroid coordinates of the current object.
    
        rectangle('Position',bb,'EdgeColor','r','LineWidth',2)              % Draw a red rectangle around the object using its bounding box coordinates.
    
        a=text(bc(1)+15,bc(2), strcat('X:', num2str(round(bc(1))), '; Y:', num2str(round(bc(2))))); % Add text label to the object with its centroid coordinates.
    
        set(a, 'FontName', 'Arial', 'FontWeight', 'bold', 'FontSize', 12, 'Color', 'yellow');       % Set font properties for the text label.
    
        q=[bc(2),bc(1),0,0]*pi/180;                                         % Convert the centroid coordinates to robot joint angles.
    
        t=fkine(r3bot,q);                                                   % Compute the forward kinematics of the robot given the joint angles.
    
        figure(f2)                                                          % Create a new figure window with handle f2.
    
        r3bot.plot(q);                                                      % Plot the robot in its current configuration. 
    end 
 
end
