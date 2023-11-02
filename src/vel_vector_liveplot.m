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
firstVertex = [52; 52; 95];
% secondVertex = [0; -199.99; 95];
secondVertex = [145; 3; 212];
thirdVertex = [52; -38; 42];

% Task Space Coefficients
vertex1_x_coefficients = trajPlanner.quintic_traj(0, desired_duration, 0, 0, 0, 0, thirdVertex(1,1), firstVertex(1,1))';
vertex1_y_coefficients = trajPlanner.quintic_traj(0, desired_duration, 0, 0, 0, 0, thirdVertex(2,1), firstVertex(2,1))';
vertex1_z_coefficients = trajPlanner.quintic_traj(0, desired_duration, 0, 0, 0, 0, thirdVertex(3,1), firstVertex(3,1))';

vertex2_x_coefficients = trajPlanner.quintic_traj(0, desired_duration, 0, 0, 0, 0, firstVertex(1,1), secondVertex(1,1))';
vertex2_y_coefficients = trajPlanner.quintic_traj(0, desired_duration, 0, 0, 0, 0, firstVertex(2,1), secondVertex(2,1))';
vertex2_z_coefficients = trajPlanner.quintic_traj(0, desired_duration, 0, 0, 0, 0, firstVertex(3,1), secondVertex(3,1))';

vertex3_x_coefficients = trajPlanner.quintic_traj(0, desired_duration, 0, 0, 0, 0, secondVertex(1,1), thirdVertex(1,1))';
vertex3_y_coefficients = trajPlanner.quintic_traj(0, desired_duration, 0, 0, 0, 0, secondVertex(2,1), thirdVertex(2,1))';
vertex3_z_coefficients = trajPlanner.quintic_traj(0, desired_duration, 0, 0, 0, 0, secondVertex(3,1), thirdVertex(3,1))';

% Task Space Coefficients
vertex1_ts_coefficients = [vertex1_x_coefficients; vertex1_y_coefficients; vertex1_z_coefficients];
vertex2_ts_coefficients = [vertex2_x_coefficients; vertex2_y_coefficients; vertex2_z_coefficients];
vertex3_ts_coefficients = [vertex3_x_coefficients; vertex3_y_coefficients; vertex3_z_coefficients];

% Setting the robot's position to the last vertex of the triangle so it'll
% start its's trajectory around the triangle on the point that it ends on
robot.servo_jp(robot.ik3001(thirdVertex));
pause(2);

% Task Space Functions for moving the robot
v3_1 = robot.run_trajectory(vertex1_ts_coefficients, desired_duration, true, model);
v1_2 = robot.run_trajectory(vertex2_ts_coefficients, desired_duration, true, model);
v2_3 = robot.run_trajectory(vertex3_ts_coefficients, desired_duration, true, model);

v1_2(:, 16) = v1_2(:, 16) + v3_1(end, 16);
v2_3(:, 16) = v2_3(:, 16) + v3_1(end, 16) + v1_2(end, 16);

motion = [v3_1; v1_2; v2_3];

% plot linear velocities
figure(2);
subplot(2, 2, 1);
hold on
plot(motion(:, 16), motion(:, 10:12));
plot(motion(:, 16), motion(:, 10:12));
plot(motion(:, 16), motion(:, 10:12));
hold off
title("Linear velocity vs time");
ylabel("Velocity (mm/s)");
xlabel("Time (s)");
legend(["Vx" "Vy" "Vz"])

% plot angular velocities
subplot(2, 2, 2);
hold on
plot(motion(:, 16), motion(:, 13:15));
plot(motion(:, 16), motion(:, 13:15));
plot(motion(:, 16), motion(:, 13:15));
hold off
title("Angular velocity vs time");
ylabel("Angular velocity (rad/s)");
xlabel("Time (s)");
legend(["Wx" "Wy" "Wz"])

% plot scalar velocity
scalar = sqrt(motion(:,10).^2 + motion(:, 11).^2 + motion(:, 12).^2);
subplot(2, 2, 3);
plot(motion(:,16), scalar);
title("Scalar velocity vs time");
ylabel("Velocity (mm/s)");
xlabel("Time (s)");

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

