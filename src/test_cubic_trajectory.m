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

vertex1_Joint1_coefficients = trajPlanner.cubic_traj(desired_duration, 0, 0, vertex3_angles(1,1), vertex1_angles(1,1))';
vertex1_Joint2_coefficients = trajPlanner.cubic_traj(desired_duration, 0, 0, vertex3_angles(2,1), vertex1_angles(2,1))';
vertex1_Joint3_coefficients = trajPlanner.cubic_traj(desired_duration, 0, 0, vertex3_angles(3,1), vertex1_angles(3,1))';

vertex2_Joint1_coefficients = trajPlanner.cubic_traj(desired_duration, 0, 0, vertex1_angles(1,1), vertex2_angles(1,1))';
vertex2_Joint2_coefficients = trajPlanner.cubic_traj(desired_duration, 0, 0, vertex1_angles(2,1), vertex2_angles(2,1))';
vertex2_Joint3_coefficients = trajPlanner.cubic_traj(desired_duration, 0, 0, vertex1_angles(3,1), vertex2_angles(3,1))';

vertex3_Joint1_coefficients = trajPlanner.cubic_traj(desired_duration, 0, 0, vertex2_angles(1,1), vertex3_angles(1,1))';
vertex3_Joint2_coefficients = trajPlanner.cubic_traj(desired_duration, 0, 0, vertex2_angles(2,1), vertex3_angles(2,1))';
vertex3_Joint3_coefficients = trajPlanner.cubic_traj(desired_duration, 0, 0, vertex2_angles(3,1), vertex3_angles(3,1))';

% Task Space Calculations        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

vertex1_x_coefficients = trajPlanner.cubic_traj(desired_duration, 0, 0, thirdVertex(1,1), firstVertex(1,1))';
vertex1_y_coefficients = trajPlanner.cubic_traj(desired_duration, 0, 0, thirdVertex(2,1), firstVertex(2,1))';
vertex1_z_coefficients = trajPlanner.cubic_traj(desired_duration, 0, 0, thirdVertex(3,1), firstVertex(3,1))';

vertex2_x_coefficients = trajPlanner.cubic_traj(desired_duration, 0, 0, firstVertex(1,1), secondVertex(1,1))';
vertex2_y_coefficients = trajPlanner.cubic_traj(desired_duration, 0, 0, firstVertex(2,1), secondVertex(2,1))';
vertex2_z_coefficients = trajPlanner.cubic_traj(desired_duration, 0, 0, firstVertex(3,1), secondVertex(3,1))';

vertex3_x_coefficients = trajPlanner.cubic_traj(desired_duration, 0, 0, secondVertex(1,1), thirdVertex(1,1))';
vertex3_y_coefficients = trajPlanner.cubic_traj(desired_duration, 0, 0, secondVertex(2,1), thirdVertex(2,1))';
vertex3_z_coefficients = trajPlanner.cubic_traj(desired_duration, 0, 0, secondVertex(3,1), thirdVertex(3,1))';


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

% Joint Space Functions for running the robot
j3_1 = robot.run_trajectory(vertex1_coefficients, desired_duration, false);
j1_2 = robot.run_trajectory(vertex2_coefficients, desired_duration, false);
j2_3 = robot.run_trajectory(vertex3_coefficients, desired_duration, false);

% Task Space Functions for running the robot
v3_1 = robot.run_trajectory(vertex1_ts_coefficients, desired_duration, true);
v1_2 = robot.run_trajectory(vertex2_ts_coefficients, desired_duration, true);
v2_3 = robot.run_trajectory(vertex3_ts_coefficients, desired_duration, true);

% Data collection for the graphs
v1_2(:,10) = v1_2(:,10) + desired_duration;
v2_3(:,10) = v2_3(:,10) + 2*desired_duration;
j1_2(:,10) = j1_2(:,10) + desired_duration;
j2_3(:,10) = j2_3(:,10) + 2*desired_duration;
motion = [v3_1; v1_2; v2_3];
jointMotion = [j3_1; j1_2; j2_3];

% Position Graph for Cubic Motion in Task Space
figure(1);
subplot(2, 2, 1);
plot(motion(:, 10), motion(:, 1:3));
title("End effector position vs time");
ylabel("position (mm)");
xlabel("time (s)");
legend(["X" "Y" "Z"])

% Velocity Graph for Cubic Motion in Task Space
subplot(2, 2, 2);
plot(motion(:, 10), motion(:, 4:6));
title("End effector velocity vs time");
ylabel("velocity (mm/s)");
xlabel("time (s)");
legend(["X" "Y" "Z"])

% Acceleration Graph for Cubic Motion in Task Space
subplot(2, 2, 3);
plot(motion(:, 10), motion(:, 7:9));
title("End effector acceleration vs time");
ylabel("acceleration (deg/s^2)");
xlabel("time (s)");
legend(["X" "Y" "Z"])

% Position Graph for Cubic Motion in Joint Space
figure(2);
subplot(2, 2, 1);
plot(jointMotion(:, 10), jointMotion(:, 1:3));
title("End effector position vs time");
ylabel("position (deg)");
xlabel("time (s)");
legend(["X" "Y" "Z"])

% Velocity Graph for Cubic Motion in Joint Space
subplot(2, 2, 2);
plot(jointMotion(:, 10), jointMotion(:, 4:6));
title("End effector velocity vs time");
ylabel("velocity (deg/s)");
xlabel("time (s)");
legend(["X" "Y" "Z"])

% Acceleration Graph for Cubic Motion in Joint Space
subplot(2, 2, 3);
plot(jointMotion(:, 10), jointMotion(:, 7:9));
title("End effector acceleration vs time");
ylabel("acceleration (deg/s^2)");
xlabel("time (s)");
legend(["X" "Y" "Z"])

% Extracting data from the .csv files
data1 = readmatrix("JointsTimesAndPose1.csv");
data2 = readmatrix("JointsTimesAndPose2.csv");
data3 = readmatrix("JointsTimesAndPose3.csv");

% Extracting matrix data and putting it into idnvidiual matrices, broken
% up by the journey from each point (i.e. 1 -> 0 to 1, 2 -> 1 to 2)
timeStamp1 = data1(:, 4);
jointsP1 = data1(:, 1:3);
eePos1 = data1(:, 5:7);
timeStamp2 = data2(:, 4);
jointsP2 = data2(:, 1:3);
eePos2 = data2(:, 5:7);
timeStamp3 = data3(:, 4);
jointsP3 = data3(:, 1:3);
eePos3 = data3(:, 5:7);

% Fixing how the end effector values are stored for plot b
eeXPos = [eePos1(:, 1); eePos2(:, 1); eePos3(:, 1)];
eeYPos = [eePos1(:, 2); eePos2(:, 2); eePos3(:, 2)];
eeZPos = [eePos1(:, 3); eePos2(:, 3); eePos3(:, 3)];

% 3D Plot of the End Effector's positon
figure(3);
hold on
plot3(motion(:,1), motion(:,2), motion(:,3));
plot3(jointMotion(:,1), jointMotion(:,2), jointMotion(:,3));
plot3(eeXPos, eeYPos, eeZPos);
title("End effector position");
xlabel("x position (mm)")
ylabel("y position (mm)")
zlabel("z position (mm)")
legend(["Task Space" "Joint Space" "No Trajectory Planning"])
hold off
view(3)

robot.shutdown()