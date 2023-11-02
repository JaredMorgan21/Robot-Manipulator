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
  
    jointsHome = [0;0;0];
    jointsNest = [-90;86.14;33.71];
    jointsArb2 = [20; 30; 30];

    expectedFinal = robot.fk3001(jointsArb2);
    positions = zeros(4,4,5);
    hold on
    for i = 1:5
        robot.servo_jp(jointsArb2);
        pause(0.5);
        
%         This has to be transposed because position is stored in row form
        position = robot.fk3001(robot.measured_js(true, false)');

        robot.servo_jp(jointsHome);
        pause(0.5);
        positions(:,:,i) = position(:,:);

        x = position(1,4);
        y = position(2,4);
        z = position(3,4);
        p = plot3(x, y, z, "--o");
        p.MarkerFaceColor = [rand(), rand(), rand()];
        view(3);

        pause(0.5);
    end
    hold off
    grid()
    legend("First run", "Second run", "Third run", "Fourth run", "Fifth run");
    
    deltaPosition = positions - expectedFinal;
    rmsPos = rms(deltaPosition, 3);
    avgPos = mean(positions, 3);
    

catch exception
    getReport(exception)
    disp('Exited on error, clean shutdown');
end

% Clear up memory upon termination
robot.shutdown()