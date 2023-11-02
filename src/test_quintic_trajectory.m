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

% Making the matrices that store the coordinates of the vertices of the
% triangle
firstVertex = [128; 132; 126];
secondVertex = [145; 3; 212];
thirdVertex = [52; -38; 42];
desired_duration = 2;

% Getting the joint angles of the robot arm for the vertices of the triangle
vertex1_angles = robot.ik3001(firstVertex)
vertex2_angles = robot.ik3001(secondVertex)
vertex3_angles = robot.ik3001(thirdVertex)


% Joint Space Calculations        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

vertex1_Joint1_coefficients = trajPlanner.quintic_traj(0, desired_duration, 0, 0, 0, 0, vertex3_angles(1,1), vertex1_angles(1,1))';
vertex1_Joint2_coefficients = trajPlanner.quintic_traj(0, desired_duration, 0, 0, 0, 0, vertex3_angles(2,1), vertex1_angles(2,1))';
vertex1_Joint3_coefficients = trajPlanner.quintic_traj(0, desired_duration, 0, 0, 0, 0, vertex3_angles(3,1), vertex1_angles(3,1))';

vertex2_Joint1_coefficients = trajPlanner.quintic_traj(0, desired_duration, 0, 0, 0, 0, vertex1_angles(1,1), vertex2_angles(1,1))';
vertex2_Joint2_coefficients = trajPlanner.quintic_traj(0, desired_duration, 0, 0, 0, 0, vertex1_angles(2,1), vertex2_angles(2,1))';
vertex2_Joint3_coefficients = trajPlanner.quintic_traj(0, desired_duration, 0, 0, 0, 0, vertex1_angles(3,1), vertex2_angles(3,1))';

vertex3_Joint1_coefficients = trajPlanner.quintic_traj(0, desired_duration, 0, 0, 0, 0, vertex2_angles(1,1), vertex3_angles(1,1))';
vertex3_Joint2_coefficients = trajPlanner.quintic_traj(0, desired_duration, 0, 0, 0, 0, vertex2_angles(2,1), vertex3_angles(2,1))';
vertex3_Joint3_coefficients = trajPlanner.quintic_traj(0, desired_duration, 0, 0, 0, 0, vertex2_angles(3,1), vertex3_angles(3,1))';

% Task Space Calculations        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

vertex1_x_coefficients = trajPlanner.quintic_traj(0, desired_duration, 0, 0, 0, 0, thirdVertex(1,1), firstVertex(1,1))';
vertex1_y_coefficients = trajPlanner.quintic_traj(0, desired_duration, 0, 0, 0, 0, thirdVertex(2,1), firstVertex(2,1))';
vertex1_z_coefficients = trajPlanner.quintic_traj(0, desired_duration, 0, 0, 0, 0, thirdVertex(3,1), firstVertex(3,1))';

vertex2_x_coefficients = trajPlanner.quintic_traj(0, desired_duration, 0, 0, 0, 0, firstVertex(1,1), secondVertex(1,1))';
vertex2_y_coefficients = trajPlanner.quintic_traj(0, desired_duration, 0, 0, 0, 0, firstVertex(2,1), secondVertex(2,1))';
vertex2_z_coefficients = trajPlanner.quintic_traj(0, desired_duration, 0, 0, 0, 0, firstVertex(3,1), secondVertex(3,1))';

vertex3_x_coefficients = trajPlanner.quintic_traj(0, desired_duration, 0, 0, 0, 0, secondVertex(1,1), thirdVertex(1,1))';
vertex3_y_coefficients = trajPlanner.quintic_traj(0, desired_duration, 0, 0, 0, 0, secondVertex(2,1), thirdVertex(2,1))';
vertex3_z_coefficients = trajPlanner.quintic_traj(0, desired_duration, 0, 0, 0, 0, secondVertex(3,1), thirdVertex(3,1))';

% Concatinating the matrices stroing the coefficients of each axis/joint for
% each point into a matrix with all of the coefficients for each joint or
% axis for each point

% Joint Space Coefficients
vertex1_coefficients = [vertex1_Joint1_coefficients; vertex1_Joint2_coefficients; vertex1_Joint3_coefficients];
vertex2_coefficients = [vertex2_Joint1_coefficients; vertex2_Joint2_coefficients; vertex2_Joint3_coefficients];
vertex3_coefficients = [vertex3_Joint1_coefficients; vertex3_Joint2_coefficients; vertex3_Joint3_coefficients];

% Task Space Coefficients
vertex1_ts_coefficients = [vertex1_x_coefficients; vertex1_y_coefficients; vertex1_z_coefficients];
vertex2_ts_coefficients = [vertex2_x_coefficients; vertex2_y_coefficients; vertex2_z_coefficients];
vertex3_ts_coefficients = [vertex3_x_coefficients; vertex3_y_coefficients; vertex3_z_coefficients];

% Setting the robot's position to the last vertex of the triangle so it'll
% start its's trajectory around the triangle on the point that it ends on
robot.servo_jp(robot.ik3001(thirdVertex));
pause(2);

%Joint Space Functions for moving the robot
%robot.run_trajectory(vertex1_coefficients, desired_duration, false);
%robot.run_trajectory(vertex2_coefficients, desired_duration, false);
%robot.run_trajectory(vertex3_coefficients, desired_duration, false);

% Task Space Functions for moving the robot
v3_1 = robot.run_trajectory(vertex1_ts_coefficients, desired_duration, true);
v1_2 = robot.run_trajectory(vertex2_ts_coefficients, desired_duration, true);
v2_3 = robot.run_trajectory(vertex3_ts_coefficients, desired_duration, true);

% Collecting some data for the graph
v1_2(:,10) = v1_2(:,10) + desired_duration;
v2_3(:,10) = v2_3(:,10) + 2*desired_duration;
motion = [v3_1; v1_2; v2_3];

% Position subplot
figure(1);
subplot(2, 2, 1);
plot(motion(:, 10), motion(:, 1:3));
title("End effector position vs time");
ylabel("position (mm)");
xlabel("time (s)");
legend(["X" "Y" "Z"])

% Velocity subplot
subplot(2, 2, 2);
plot(motion(:, 10), motion(:, 4:6));
title("End effector velocity vs time");
ylabel("velocity (mm/s)");
xlabel("time (s)");
legend(["X" "Y" "Z"])

% Acceleration subplot
subplot(2, 2, 3);
plot(motion(:, 10), motion(:, 7:9));
title("End effector acceleration vs time");
ylabel("acceleration (mm/s^2)");
xlabel("time (s)");
legend(["X" "Y" "Z"])

% 3D Plot of the End Effector's Position
figure(2);
plot3(motion(:,1), motion(:,2), motion(:,3));
title("End effector position");
xlabel("x position (mm)")
ylabel("y position (mm)")
zlabel("z position (mm)")

robot.shutdown()