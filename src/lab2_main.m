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
model = Model(robot);
try

  %Travel to the first point on the triangle
  robot.servo_jp([0; 40; 0]);


  SERV_ID = 1848;            % we will be talking to server ID 1848 on
                           % the Nucleo
  SERVER_ID_READ =1910;% ID of the read packet
  DEBUG   = true;          % enables/disables debug prints
  
  %Hardcoded DH Table
  dhTable = [0        95      0       -90;
              -90       0       100     0;
              90        0       100     0];

  %The 4 points for the RMSE data collection
  joint_pos = [ [60 30 0];
                [-60  -30  10];
                [45 -45 40];
                [90 45 -50];];

  %The 3 point for the triangle in part 8
  joint_part8 = [ [0 -30 21];
                [0  30  42];
                [0 40 0];];

  %Initializing matrices to store the measured data
  joints = zeros(5000, 3);
  ee_pos = zeros(5000, 3);
  timestamp = zeros(5000, 1);

  %For loop to run through each of the points of the triangle while
  %collecting data
  for i =1:3
      %Travel to the given point via interpolation
      robot.interpolate_jp(joint_part8(i, :).', 5000);
      tic
      %Read data over the course of 5 seconds
      for k = 1:5000
          %Record joint angles
          joint = robot.measured_js(true, false);
          %Record cartesian coordinates of the end effector
          pos = robot.fk3001(joint(1,:)')
          %Extract the relevant data
          joints(k, :) = joint(1,:);
          ee_pos(k, :) = pos(1:3,4);
          timestamp(k) = toc*1000;
          pause(0.001);
      end   

      %Put all of the data collected into one matrix
      jointsTimestepAndEEPos = [joints timestamp ee_pos];

      %Write that matrix data to a csv based off of which points the robot
      %is moving to and from-- if we didn't have this if statement, the for
      %loop would overwrite all of the previous data while travelling to
      %new points
      if i == 1
        writematrix(jointsTimestepAndEEPos, "Lab2Part8Point1.csv");
      elseif i == 2
        writematrix(jointsTimestepAndEEPos, "Lab2Part8Point2.csv");
      else
        writematrix(jointsTimestepAndEEPos, "Lab2Part8Point3.csv");
      end      
     
      %while toc < 5
      %  jointPart8 = robot.measured_js(true, false).';
      %  pause(0.1)
      %  model.plot_arm(jointPart8(:,1))
      %end
  end
    
  %For RMSE Data Collection
  for i =1:4 
      robot.interpolate_jp(joint_pos(i, :).', 5000);
      tic
    
      while toc < 5
        currJointPos = robot.measured_js(true, false).';
        pause(0.1)
        model.plot_arm(currJointPos(:,1))
      end
      disp(robot.fk3001(joint_pos(i, :).'))
      toc
  end

  %Extracting data from the .csv files, with each matrix being the data
  %collected while travelling from one point to the other
  data1 = readmatrix("Lab2Part8Point1.csv");
  data2 = readmatrix("Lab2Part8Point2.csv");
  data3 = readmatrix("Lab2Part8Point3.csv");

  %Extracting matrix data and putting it into idnvidiual matrices, broken
  %up by the journey from each point (i.e. 1 -> 0 to 1, 2 -> 1 to 2)
  timeStamp1 = data1(:, 4);
  jointsP1 = data1(:, 1:3);
  eePos1 = data1(:, 5:7);

  timeStamp2 = data2(:, 4);
  jointsP2 = data2(:, 1:3);
  eePos2 = data2(:, 5:7);

  timeStamp3 = data3(:, 4);
  jointsP3 = data3(:, 1:3);
  eePos3 = data3(:, 5:7);

  %Intermediary time steps-- making each timeStamp with resepect to the
  %when the robot arm left the nested position instead of the last
  %position
  newTimeStamp2 = timeStamp1(5000, 1) + timeStamp2;
  newTimeStamp3 = newTimeStamp2(5000, 1) + timeStamp3;

  %Putting all of the timestamps together
  totalTime = [timeStamp1; newTimeStamp2; newTimeStamp3];

  %Fixing how the joint values are stored for plot a
  newJointsQ1 = [jointsP1(:, 1); jointsP2(:, 1); jointsP3(:, 1)];
  newJointsQ2 = [jointsP1(:, 2); jointsP2(:, 2); jointsP3(:, 2)];
  newJointsQ3 = [jointsP1(:, 3); jointsP2(:, 3); jointsP3(:, 3)];

  %Fixing how the end effector values are stored for plot b
  eeXPos = [eePos1(:, 1); eePos2(:, 1); eePos3(:, 1)];
  eeYPos = [eePos1(:, 2); eePos2(:, 2); eePos3(:, 2)];
  eeZPos = [eePos1(:, 3); eePos2(:, 3); eePos3(:, 3)];
  
  %Plotting Data for Part 8
  %Plotting A
  figure(1)
  hold on
  title("Joint Positions vs Time");
  xlabel("Time (s)");
  ylabel("Joint Position (degrees)");
  plot(totalTime, newJointsQ1(:,1));
  plot(totalTime, newJointsQ2(:,1));
  plot(totalTime, newJointsQ3(:,1));
  xlim([0,23000]);
  ylim([-40, 50]);
  hold off
  legend("Joint 1", "Joint 2", "Joint 3");
  view(2);

  %Plotting B
  figure(2)
  hold on
  title("End Effector X and Z Position vs Time");
  xlabel("Time (s)");
  ylabel("End Effector Coordinate (mm)");
  plot(totalTime, eeXPos);
  plot(totalTime, eeZPos);
  xlim([totalTime(1,1), totalTime(end, 1)]);
  ylim([0, 200]);
  hold off
  view(2);
  legend("X Coordinate", "Y Coordinate");

  %Plotting C
  figure(3)
  hold on
  title("End Effector X Position vs. Z Position");
  xlabel("X Position (mm)");
  ylabel("Z Position (mm)");
  plot(eeXPos, eeZPos);
  xlim([min(eeXPos), max(eeXPos)]);
  ylim([min(eeZPos), max(eeZPos)]);
  view(2);
  hold off

  %Plotting D
  figure(4)
  hold on
  title("End Effector X Position vs Y Position");
  xlabel("X Position (mm)");
  ylabel("Y Position (mm)");
  plot (eeXPos, eeYPos);
  xlim([min(eeXPos), max(eeXPos)]);
  ylim([min(eeYPos), max(eeYPos)]);
  view(2);
  hold off;

catch exception
    getReport(exception)
    disp('Exited on error, clean shutdown');
end

% Clear up memory upon termination
robot.shutdown()