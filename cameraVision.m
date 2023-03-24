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
    
    % Display the image
    figure(f1)
    imshow(data)
    
    
    
    %This is a loop to bound the red objects in a rectangular box.
    for object = 1:length(stats)
      
        bb = stats(object).BoundingBox;
        bc = stats(object).Centroid;
        rectangle('Position',bb,'EdgeColor','r','LineWidth',2)
     
     
      
        a=text(bc(1)+15,bc(2), strcat('X: ', num2str(round(bc(1))), '    Y: ', num2str(round(bc(2)))));
        set(a, 'FontName', 'Arial', 'FontWeight', 'bold', 'FontSize', 12, 'Color', 'yellow');
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
