
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

% Initializes classes used in program
robot = Robot(myHIDSimplePacketComs);
model = Model(robot);
trajPlanner = Traj_Planner;

% Generates new camera parameters if file does not already exist
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
    ballSize = 200;
    
    % Moves arm into zero position and initializes color_processor
    color_processor = Color_Processor(cam);
    robot.servo_jp(robot.ik3001([100 0 195])');
    pause(1);
    
    % Takes image and generates masks
    img = cam.getImage();
    imgHSV = rgb2hsv(img);

    royMask = color_processor.mask_HSV(imgHSV, 0, 90, 100, 255, 255, 255);
    gMask = color_processor.mask_HSV(imgHSV, 100, 70, 50, 130, 255, 255);
    otherMask = color_processor.mask_HSV(imgHSV, 0, 100, 120, 255, 255, 255);

    mask = royMask | gMask | otherMask;
    
    % Filters image and draws centriods
    imgFiltered = color_processor.filter_img(img, mask);
    imgDraw = color_processor.draw_centroids(img, mask, ballSize);
    figure(2);
    imshow(imgFiltered);
%     imshow(imgDraw)
    
    % Locates centriods and grabs balls until none are detected
    centroids = color_processor.find_centroids(mask, ballSize);
    while height(centroids) > 0
        robot.grab_ball(2, true, color_processor, cam, trajPlanner, model, mask, imgHSV, ballSize);
        
        % Updates image after ball is grabbed
        centroids = color_processor.find_centroids(mask, ballSize);
        img = cam.getImage();
        imgHSV = rgb2hsv(img);

        royMask = color_processor.mask_HSV(imgHSV, 0, 90, 100, 255, 255, 255);
        gMask = color_processor.mask_HSV(imgHSV, 100, 70, 50, 130, 255, 255);
        otherMask = color_processor.mask_HSV(imgHSV, 0, 100, 120, 255, 255, 255);
    
        mask = royMask | gMask | otherMask;

        imgFiltered = color_processor.filter_img(img, mask);
%         figure(1);
%         imshow(imgHSV);
        figure(2);
        imshow(imgFiltered);
    end

    pause(1);
    robot.interpolate_jp([0 0 0], 1000);

    
catch exception
    fprintf('\n ERROR!!! \n \n');
    disp(getReport(exception));
    disp('Exited on error, clean shutdown');
end

%% Shutdown Procedure
robot.shutdown()