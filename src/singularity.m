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
trajPlanner = Traj_Planner;
model = Model(robot);

desired_duration = 2;

% Making the matrices that store the coordinates of the vertices of the
% triangle
firstVertex = [-45; 0; 90];
thirdVertex = [52; -38; 42];

% Task Space Coefficients
vertex1_x_coefficients = trajPlanner.quintic_traj(0, desired_duration, 0, 0, 0, 0, thirdVertex(1,1), firstVertex(1,1))';
vertex1_y_coefficients = trajPlanner.quintic_traj(0, desired_duration, 0, 0, 0, 0, thirdVertex(2,1), firstVertex(2,1))';
vertex1_z_coefficients = trajPlanner.quintic_traj(0, desired_duration, 0, 0, 0, 0, thirdVertex(3,1), firstVertex(3,1))';

% Task Space Coefficients
vertex1_ts_coefficients = [vertex1_x_coefficients; vertex1_y_coefficients; vertex1_z_coefficients];


% Setting the robot's position to the last vertex of the triangle so it'll
% start its's trajectory around the triangle on the point that it ends on
robot.servo_jp(robot.ik3001(thirdVertex));
pause(2);

% Task Space Functions for moving the robot
v3_1 = robot.run_trajectory(vertex1_ts_coefficients, desired_duration, true, model);

motion = [v3_1];

% plot end effector
figure(3);
subplot(2, 1, 1);
plot3(motion(:, 18), motion(:, 19), motion(:, 20));
xlabel("x position (mm)")
ylabel("y position (mm)")
zlabel("z position (mm)")
title("End Effector Position")

% plot jv vs time
subplot(2, 1, 2);
plot(motion(:, 16), motion(:, 17));
xlabel("time (seconds)")
ylabel("jacobian determinant")
title("Determinant of linear velocity jacobian vs time")

% Clear up memory upon termination
robot.shutdown()

