%aclear
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
model = Model(robot);
try

    % Testing Inverse Kinematics
    inverse = robot.ik3001([100; 0; 195]);
    robot.fk3001(inverse);

    % Home Position
    jointsHome = [0; 0; 0];
    posHome = [100; 0; 195];
    
    % Nest Position
    jointsNest = [-90;86.14;33.71];
    posNest = robot.fk3001(jointsNest);
    posNest = posNest(1:3,4);
    
    % Creating sample time variable for the duration of the robot's
    % interpolation
    sample_time = 500;
    
    % Creating the matrices storing the x, y, and z coordinates
    firstVertex = [128; 132; 126];
    secondVertex = [145; 3; 212];
    thirdVertex = [52; -38; 42];

    % Creating two arbitrary positions for validating inverse kinematics
    posArb = firstVertex;
    posArb2 = secondVertex;

    % Getting the joint angles of the vertices of the triangles
    firstJoint = robot.ik3001(firstVertex);
    secondJoint = robot.ik3001(secondVertex);
    thirdJoint = robot.ik3001(thirdVertex);

    % Setting the robot's position to the last vertex of the triangle so it'll
    % start its's trajectory around the triangle on the point that it ends on
    robot.servo_jp(thirdJoint');
    
    % Storing the triangle vertices in one matrix
    triangle_vertices = [firstVertex secondVertex thirdVertex];
    triangle_joints = [firstJoint secondJoint thirdJoint];

    % Validating ik3001() with fk3001()
    posHome;
    inverseHome = robot.ik3001(posHome);
    calcHome = robot.fk3001(inverseHome);

    posNest;
    inverseNest = robot.ik3001(posNest);
    calcNest = robot.fk3001(inverseNest);

    posArb;
    inverseArb = robot.ik3001(posArb);
    calcArb = robot.fk3001(inverseArb);

    posArb2;
    inverseArb2 = robot.ik3001(posArb2);
    calcArb2 = robot.fk3001(inverseArb2);

    % Setting the robot's position to the last vertex of the triangle so it'll
    % start its's trajectory around the triangle on the point that it ends on
    sample_time = 500;    

    % Creating the vertices of the triangles
    firstVertex = [128; 132; 126];
    secondVertex = [145; 3; 212];
    thirdVertex = [52; -38; 42];

    % Getting the joint angles of the robot arm at each point of the
    % triangle
    firstJoint = robot.ik3001(firstVertex);
    secondJoint = robot.ik3001(secondVertex);
    thirdJoint = robot.ik3001(thirdVertex);

    % Setting the robot's position to the last vertex of the triangle so it'll
    % start its's trajectory around the triangle on the point that it ends on
    robot.servo_jp(thirdJoint');
    
    % Concatinating the vertices into one matrix
    triangle_vertices = [firstVertex secondVertex thirdVertex];
    triangle_joints = [firstJoint secondJoint thirdJoint];

    % Data Collection Loop
    pause(1);
    for i = 1:3
        joint_angles = zeros(sample_time, 3);
        ee_pos = zeros(sample_time, 3);
        timestamp = zeros(sample_time, 1);

        % Need to swtich for part five of part 3 of the lab
        %robot.interpolate_jp(triangle_joints(:,i)', sample_time);
        robot.servo_jp(triangle_joints(:,i)');

        tic
        j = 1;
        while toc * 1000 < sample_time
            measured_joint_angles = robot.measured_js(true, false);
            joints = measured_joint_angles(1,:);

            figure(1);
            model.plot_arm(joints')

            pos = robot.fk3001(joints');
            joint_angles(j,:) = joints;

            ee_pos(j, :) = pos(1:3, 4);
            timestamp(j) = toc*1000 + (i-1) * 5000;
            pause(.01);
            j = j + 1;
        end

        joint_angles = joint_angles(1:j-1,:);
        timestamp = timestamp(1:j-1, :);
        ee_pos = ee_pos(1:j-1,:);

        joints_time_eePos = [joint_angles timestamp ee_pos];

        if i==1
            writematrix(joints_time_eePos, "JointsTimesAndPose1.csv");
        elseif i==2
            writematrix(joints_time_eePos, "JointsTimesAndPose2.csv");
        elseif i==3
            writematrix(joints_time_eePos, "JointsTimesAndPose3.csv");
        end
        toc
    end

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

    % Putting all of the timestamps together
    totalTime = [timeStamp1; timeStamp2; timeStamp3];

    % Fixing how the joint values are stored for plot a
    newJointsQ1 = [jointsP1(:, 1); jointsP2(:, 1); jointsP3(:, 1)];
    newJointsQ2 = [jointsP1(:, 2); jointsP2(:, 2); jointsP3(:, 2)];
    newJointsQ3 = [jointsP1(:, 3); jointsP2(:, 3); jointsP3(:, 3)];

    % Fixing how the end effector values are stored for plot b
    eeXPos = [eePos1(:, 1); eePos2(:, 1); eePos3(:, 1)];
    eeYPos = [eePos1(:, 2); eePos2(:, 2); eePos3(:, 2)];
    eeZPos = [eePos1(:, 3); eePos2(:, 3); eePos3(:, 3)];

    totalTime = totalTime(totalTime ~= 0);

    % Plotting X, Y, and Z Position vs Time
    figure(2);
    title("X, Y, and Z Position of the End-Effector vs Time");
    hold on
    plot(totalTime, eeXPos);
    plot(totalTime, eeYPos);
    plot(totalTime, eeZPos);
    xlabel("Time (s)");
    ylabel("X, Y, and Z Coordinate (mm)");
    hold off
    legend("X Position", "Y Position", "Z Position");
    view(2);

    % Plotting X, Y, and Z coordinates of the End Effector
    figure(3);
    title("X, Y, and Z coordinates of the End Effector");
    xlabel("X Position (mm)");
    ylabel("Y Position (mm)");
    zlabel("Z Position (mm)");
    hold on
    plot3(eeXPos, eeYPos, eeZPos);
    hold off
    view(3);

    % Plotting Joint Angles of the Arm as it Travels Along a Triangle vs
    % Time
    figure(4);
    title("Joint Angles of the Arm vs Time");
    xlabel("Time (ms)");
    ylabel("Joint Angles")
    hold on
    plot(totalTime, newJointsQ1);
    plot(totalTime, newJointsQ2);
    plot(totalTime, newJointsQ3);
    hold off
    legend("Joint 1", "Joint 2", "Joint 3");
    view(2);
    
catch exception
    getReport(exception)
    disp('Exited on error, clean shutdown');
end

% Clear up memory upon termination
robot.shutdown()