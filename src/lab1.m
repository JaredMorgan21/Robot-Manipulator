%%
% RBE3001 - Laboratory 1 
% 
% Instructions
% ------------
% Welcome again! This MATLAB script is your starting point for Lab
% 1 of RBE3001. The sample code below demonstrates how to establish
% communication between this script and the Nucleo firmware, send
% setpoint commands and receive sensor data.
% 
% IMPORTANT - understanding the code below requires being familiar
% with the Nucleo firmware. Read that code first.

% Lines 15-37 perform necessary library initializations. You can skip reading
% to line 38.
clear
clear java
clear classes;

vid = hex2dec('16c0');
pid = hex2dec('0486');

disp (vid);
disp (pid);

javaaddpath ../lib/SimplePacketComsJavaFat-0.6.4.jar;
import edu.wpi.SimplePacketComs.*;
import edu.wpi.SimplePacketComs.device.*;
import edu.wpi.SimplePacketComs.phy.*;
import java.util.*;
import org.hid4java.*;
version -java
myHIDSimplePacketComs=HIDfactory.get();
myHIDSimplePacketComs.setPid(pid);
myHIDSimplePacketComs.setVid(vid);
myHIDSimplePacketComs.connect();

% Create a PacketProcessor object to send data to the nucleo firmware
robot = Robot(myHIDSimplePacketComs); 
try
  SERV_ID = 1848;            % we will be talking to server ID 1848 on
                           % the Nucleo
  SERVER_ID_READ =1910;% ID of the read packet
  DEBUG   = true;          % enables/disables debug prints

  % Instantiate a packet - the following instruction allocates 60
  % bytes for this purpose. Recall that the HID interface supports
  % packet sizes up to 64 bytes.
  packet = zeros(15, 1, 'single');
  joints = zeros(5000, 3);
  timestamp = zeros(5000, 1);
   
  % Sets robot to the "zero" position
  homePts = [0, 0, 0];
  robot.servo_jp(homePts);
  pause(3);
  
  % Reads values as arm is moving to goal position
  tic
  goalPts = [31.44, 49.94, 11.15];
  robot.servo_jp(goalPts);
  for k = 1:5000
    joint = robot.measured_js(true, false);
    joints(k,:) = joint(1, :);
    timestamp(k) = toc*1000;
    pause(0.001);
  end
  pause(1)

%   [uniqueJoints, colIdx, rowIdx] = unique(joints, "rows");
%   uniqueTimestamp = timestamp(colIdx);

  % Combines values into matrix and writes to a .csv file
  jointsAndTimestep = [joints timestamp];

  writematrix(jointsAndTimestep , "interpolateQuestion5Position2NonInterpolated.csv");
  
catch exception
    getReport(exception)
    disp('Exited on error, clean shutdown');
end

% Clear up memory upon termination
robot.shutdown()

toc