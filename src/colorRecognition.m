
%%
% RBE 3001 Lab 5 example code!
%%
clc;
clear;
clear java;
format short

%% Flags
DEBUG = false;
STICKMODEL = false;

%% Setup
vid = hex2dec('16c0');
pid = hex2dec('0486');

if DEBUG
    disp(vid);
    disp(pid);
end


javaaddpath ../lib/SimplePacketComsJavaFat-0.6.4.jar;
import edu.wpi.SimplePacketComs.*;
import edu.wpi.SimplePacketComs.device.*;
import edu.wpi.SimplePacketComs.phy.*;
import java.util.*;
import org.hid4java.*;
version -java;
myHIDSimplePacketComs=HIDfactory.get();
myHIDSimplePacketComs.setPid(pid);
myHIDSimplePacketComs.setVid(vid);
myHIDSimplePacketComs.connect();

robot = Robot(myHIDSimplePacketComs);
model = Model(robot);
trajPlanner = Traj_Planner;

try
    load("camParams.mat");
    disp("Loaded Camera Parameters from camParams.mat");
catch exception
    disp("Could not find camParams.mat, creating new Camera object");
    cam = Camera();
    save("camParams.mat","cam");
    disp("Saved Camera Parameters to camParams.mat");
end

%% Main Loop
try
    % Set up camera
    if cam.params == 0
        error("No camera parameters found!");
    end
    color_processor = Color_Processor(cam);
    robot.servo_jp(robot.ik3001([100 0 195])');
    pause(1);
    
    tic
%     while toc < 10
        img = cam.getImage();
        imgHSV = rgb2hsv(img);

        mask = color_processor.mask_HSV(imgHSV, 0, 100, 100, 255, 255, 255);

%         figure(1);
%         subplot(2,2,1);
%         imshow(img);
%         subplot(2,2,2);
%         imshow(imgHSV);
%         subplot(2,2,3);
%         imshow(mask);
%         subplot(2,2,4);
%         imshow(imgFiltered);
    
%         figure(2);

%         filteredHSV = filterImg(imgHSV, mask);

        imgFiltered = color_processor.filter_img(img, mask);
        imgDraw = color_processor.draw_centroids(img, mask, 200);
        figure(2);
        imshow(imgFiltered);

        centroids = color_processor.find_centroids(mask, 200);
        robot.grab_ball(2, true, color_processor, cam, trajPlanner, model, mask, 200);

        pause(1);
        robot.interpolate_jp([0 0 0], 1000);

        toc
%     end
    
catch exception
    fprintf('\n ERROR!!! \n \n');
    disp(getReport(exception));
    disp('Exited on error, clean shutdown');
end

%% Shutdown Procedure
robot.shutdown()
cam.shutdown()