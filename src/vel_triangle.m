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
% robot = Robot(myHIDSimplePacketComs);
robot = Robot(myHIDSimplePacketComs);
% trajPlanner = Traj_Planner;
% model = Model(robot);

% Making the matrices that store the coordinates of the vertices of the
% triangle
firstVertex = [65; 52; 95];
secondVertex = [145; 3; 212];
thirdVertex = [52; -38; 42];

robot.servo_jp(robot.ik3001(firstVertex)');
pause(1);

robot.unitVector(firstVertex, secondVertex)

% Gets the velocity data of the arm as it travels between points
m12 = robot.iVel(200, firstVertex, secondVertex);
m23 = robot.iVel(200, secondVertex, thirdVertex);
m31 = robot.iVel(200, thirdVertex, firstVertex);

% Standardizes data
m23(:, 10) = m23(:, 10) + m12(end, 10);
m31(:, 10) = m31(:, 10) + m23(end, 10);

% Concatinates data in one array
motion = [m12; m23; m31];

% Creates the subplots of the data as it traces the triangle
subplot(2, 2, 1);
plot(motion(:, 10), motion(:, 1:3));
xlabel("Time (s)");
ylabel("Position (mm)");
legend(["X" "Y" "Z"]);
title("End effector position vs time");

subplot(2, 2, 2);
plot(motion(:, 10), motion(:, 4:6));
xlabel("Time (s)");
ylabel("Velocity (mm/s)");
legend(["VX" "VY" "VZ"]);
title("End effector velocity vs time");

subplot(2, 2, 3);
plot(motion(:, 10), motion(:, 7:9));
xlabel("Time (s)");
ylabel("Acceleration (mm/s^2)");
legend(["AX" "AY" "AZ"]);
title("End effector acceleration vs time");

figure(2);
plot3(motion(:,1), motion(:,2), motion(:,3));

% Clear up memory upon termination
robot.shutdown()