# raspberryColorVision
# Color Image Recognition with Kinematic Positioning and modular feed for MATLAB ®
## Synopsis
The current repository contains an array of codes focused on Color Image recognition systems based on MATLAB for use in recognition of labels with modularity in applications. It utilizes a collection of toolboxes, including Robotics Toolbox for MATLAB, Robotics, Vision & Control: 3rd edition in MATLAB, and much more listed below. 
It provides a simple interface of Figure elements of MATLAB to display color detection of elements in the provided feed, with an addition of a 3D simulation of a robotic arm that tracks the color label detected in the screen. It shows a bounding box as well as the size of the detected element.

## Code Example
The following example is based on the integrated camera code:
```ruby
clc

clear all

close all

t3r=[0 0 3 pi/2;0 0 3 0;0 0 3 0];

r3bot=SerialLink(t3r,'name','robot');

r3bot.plot([0 0 0]);

a = imaqhwinfo;

f1=figure;

f2=figure;

% Capture the video frames using the videoinput function

% You have to replace the resolution & your installed adaptor name.

objects=imaqfind;

delete(objects);

imaqmex('feature','-limitPhysicalMemoryUsage',false);

vid = videoinput('winvideo',1);

% Set the properties of the video object

set(vid, 'FramesPerTrigger', Inf);

set(vid, 'ReturnedColorspace', 'rgb')

vid.FrameGrabInterval = 1;

%start the video aquisition here

start(vid)

n=50;
```
This part is used for the configuration of the video feed, in this example the Image Aqcuisition Toolbox in conjunction with the Image Acquisition Toolbox support package for OS Generic Video Interface is used to define the video input from winvideo 1, meaning the integrated camera from a Laptop running windows.
```ruby
% Set a loop that stop after 100 frames of aquisition

vid.FramesAcquired<=100

while(1)

% Get the snapshot of the current frame

data = getsnapshot(vid);

% Now to track red objects in real time

% we have to subtract the red component

% from the grayscale image to extract the red components in the image.

diff_im = imsubtract(data(:,:,1), rgb2gray(data));

%Use a median filter to filter out noise

diff_im = medfilt2(diff_im, [3 3]);

% Convert the resulting grayscale image into a binary image.

diff_im = im2bw(diff_im,0.18);

% Remove all those pixels less than 300px

diff_im = bwareaopen(diff_im,300);

% Label all the connected components in the image.

bw = bwlabel(diff_im, 8);

% Here we do the image blob analysis.

% We get a set of properties for each labeled region.

stats = regionprops(bw, 'BoundingBox', 'Centroid');
```
The previous code is responsible for the image processing of the image as well as the loop for taking multiple images and labeling. Once that is defined the system can be displayed using figures in MATLAB

```ruby

% Display the image

figure(f1)

imshow(data)

%This is a loop to bound the red objects in a rectangular box.

for object = 1:length(stats)

bb = stats(object).BoundingBox;

bc = stats(object).Centroid;

rectangle('Position',bb,'EdgeColor','r','LineWidth',2)

a=text(bc(1)+15,bc(2), strcat('X: ', num2str(round(bc(1))), ' Y: ', num2str(round(bc(2)))));

set(a, 'FontName', 'Arial', 'FontWeight', 'bold', 'FontSize', 12, 'Color', 'yellow');
```

The previous snippet taks the element detected and gives it visual representation trough bounding boxes

```ruby
if -bc(2)<-450

q=[bc(1),-450,0]*pi/180;

elseif -bc(2)>-250

q=[bc(1),-250,0]*pi/180;

else

q=[bc(1),-bc(2),0]*pi/180;

end

t=fkine(r3bot,q);

figure(f2)

r3bot.plot(q);

end

end

% Both the loops end here.

% Stop the video aquisition.

stop(vid);

% Flush all the image data stored in the memory buffer.

flushdata(vid);

% Clear all variables

clear all

```
The last part is focused on the calculation of position and the query for the forward kinematic appliance in the previously defined robotic system.

The following screenshots display the interface for detecting red elements in the screen:
![Blue element](https://drive.google.com/file/d/1IpYhmftudXDDfuegjR9Ssl3mYEVJt5Fd/view?usp=share_link)
![Red element](https://drive.google.com/file/d/1qJHIudlaF07xhLCFoTxSFcxrVrcX_xvt/view?usp=share_link)
![Red element second position](https://drive.google.com/file/d/1FUkLg5gCm3EKkBjCqzdJc7A5uP-FbiKJ/view?usp=share_link)

## Installation
For the code to be utilized the following collection of toolboxes is necessary:

* Image Acquisition Toolbox from MathWorks
* Image Processing Toolbox from MathWorks
* Machine Vision Toolbox for MATLAB from Peter Corke
* Robotics Toolbox for MATLAB from Peter Corke
* MATLAB Support Package for Raspberry Pi Hardware (If Raspberry is used for video input) from MathWorks
* Image Acquisition Toolbox Support Package for OS Generic Video Interface from MathWorks
* MATLAB Support Package for USB Webcams from MathWorks
* MATLAB Support Package for IP Cameras (If IP Camera is used for video input) from MathWorks
* Image Acquisition Toolbox Support Package for Kinect for Windows Sensor (If Kinect is used for video input) from MathWorks

After installing the toolboxes just choose the desired video input and run it directly on MATLAB:
* cameraVision.m for integrated cameras
* cameraVisionOBS.m for OBS virtual cameras
* cameraVisionKINECT.m for Kinect sensors
* raspVideo.m for Raspberry Pi cameras

Special thanks to J. A. for his contribution.
