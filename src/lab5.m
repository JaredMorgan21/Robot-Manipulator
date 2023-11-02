
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
    imshow(cam.getImage());
    
    
catch exception
    fprintf('\n ERROR!!! \n \n');
    disp(getReport(exception));
    disp('Exited on error, clean shutdown');
end

cam.calculateCameraPos();
x_y = [525 171]

intrinsic_params = cam.getCameraInstrinsics();
pose = cam.getCameraPose();
rotation_matrix = pose(1:3,1:3);
translation_vector = pose(1:3, 4);

checkered_board_frame = pointsToWorld(intrinsic_params, rotation_matrix, translation_vector, x_y)

%robot frame to checker frame
rf_to_cf = [0 1 0 75;
            1 0 0 -100;
            0  0 -1 0;
            0  0 0  1];

base_point = rf_to_cf * [checkered_board_frame 0 1]'

%% Shutdown Procedure
robot.shutdown()
cam.shutdown()
