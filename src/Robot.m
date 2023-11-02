classdef Robot < handle
    
    properties
        myHIDSimplePacketComs;
        pol; 
        GRIPPER_ID = 1962
        goalJS = zeros(3, 1);
        SERVO_ID = 1848;
        graph = Graphing();
    end
    
    methods
        
        % The is a shutdown function to clear the HID hardware connection
        function  shutdown(self)
	    % Close the device
            self.myHIDSimplePacketComs.disconnect();
        end
        
        % Create a packet processor for an HID device with USB PID 0x007
        function self = Robot(dev)
            self.myHIDSimplePacketComs=dev; 
            self.pol = java.lang.Boolean(false);
        end

        %Calculates the Jacobian matrix based on the passed-in joint angles
        function finalMat = jacob3001(self, jointAngles)
            q1 = jointAngles(1);
            q2 = jointAngles(2);
            q3 = jointAngles(3);


            s1 = sind(q1);
            s2 = sind(q2);
            s3 = sind(q3);

            c1 = cosd(q1);
            c2 = cosd(q2);
            c3 = cosd(q3);

            l = 100;

            r1c1 = -l*s1*s2 - l*s1*c2*c3 + l*s1*s2*s3;

            r1c2 = -l*c1*c2*s3 + l*c1*c2 - l*c1*s2*c3;
            
            r1c3 = -l*c1*c2*s3 - l*c1*s2*c3;
            
            r2c1 = l*c1*s2 + l*c1*c2*c3 - l*c1*s2*s3;
            
            r2c2 = -l*s1*c2*s3 + l*s1*c2 - l*s1*s2*c3;
            
            r2c3 = -l*s1*c2*s3 - l*s1*s2*c3;
            
            r3c1 = 0;
            
            r3c2 = -l*s2 - l*c2*c3 + l*s2*s3;
            
            r3c3 = -l*c2*c3 + l*s2*s3;
            
            preMat = [r1c1 r1c2 r1c3; r2c1 r2c2 r2c3 ; r3c1 r3c2 r3c3; 0 -sind(q1) -sind(q1) ; 0 cosd(q1) cosd(q1); 1 0 0];
            detMat = preMat(1:3,:);
            if(det(detMat) < 10^5)
                error("Jacobian too small");
            end
            finalMat = preMat;
        end

        % This method takes a 3 × 1 task space position vector (i.e. x, y,
        % z components of end effector th787at is the robot end-effector s position w.r.t the base frame)
        % as the input and returns a set of corresponding joint angles (i.e. q1, q2, q3)       
        % that would make the robot s end-effector move to that target position.
        function jointAngles = ik3001(self, endEffectorPos)  
            if(endEffectorPos(1)^2 + endEffectorPos(2)^2 + endEffectorPos(3)^2 > 295^2)
                error("Position not reachable within bounds")
            end

            x = endEffectorPos(1);
            y = endEffectorPos(2);
            z = endEffectorPos(3);

            l1 = 95;
            l2 = 100;
            l3 = 100;

            q1 = atan2(y, x);

            s = z - l1;
            r = sqrt(x^2 + y^2);
            c = (r^2 + s^2 - l2^2 - l3^2) / (2*l2*l3);

            q3 = atan2(sqrt(1-c^2), c);

            gamma = atan2(s,r);
            delta = atan2(l3*sin(q3), l2 + l3*cos(q3));

            q2 = pi/2 - (gamma + delta);

            jointAngles = [rad2deg(q1) ; rad2deg(q2) ; rad2deg(q3) - 90];
        end
        
        % Takes data from measured_js() and returns a 4×4 homogeneous 
        % transformation matrix based upon the current joint positions in degrees.
        function transMatrix = measured_cp(self)
            joint_pos = self.measured_js(true, false);
            transMatrix = self.fk3001(joint_pos(1,:));
        end

        % Takes data from goal_js() and returns a 4 × 4 homogeneous transformation matrix 
        % based upon the commanded end of motion joint set point positions in degrees.
        function transMatrix = goal_cp(self)
            transMatrix = self.fk3001(self.goal_js());
        end


        % Takes data from setpoint_js() and returns a 4 × 4 homogeneous transformation 
        % matrix based upon the current joint set point positions in degrees 
        % (will return intermediate setpoint with interpolation)
        function transMatrix = setpoint_cp(self)
            setpoint_pos = self.setpoint_js();
            transMatrix = self.fk3001(setpoint_pos, dhTable);
        end

        % Creates an Ai matrix from 4 parameters representing theta, d, a,
        % alpha
        function transformMat = dh2mat(self, inArr)
            theta = inArr(1);
            d = inArr(2);
            a = inArr(3);
            alpha = inArr(4);

            transformMat = [cosd(theta) -sind(theta)*cosd(alpha)    sind(theta)*sind(alpha)     a*cosd(theta);
                            sind(theta) cosd(theta)*cosd(alpha)     -cosd(theta)*sind(alpha)    a*sind(theta);
                            0           sind(alpha)                 cosd(alpha)                 d;
                            0           0                           0                           1];
        end

        % Takes a nx4 array of DH tables and returns the final
        % transformation matrix
        function finalMat = dh2fk(self, dhTables)
            % sets the final matrix to identity matrixfk3001
            intermediateMat = eye(4);

            % multiplies subsequent Ai matrices together
            for i = 1 : height(dhTables)
                intermediateMat = intermediateMat * self.dh2mat(dhTables(i, :));
            end

            finalMat = intermediateMat;

        end

        % recieves a nx1 array of joints and a nx4 array of dh tables for 
        % the joints, returns the end effector position
        function endPos = fk3001(self, joints)
            dhTables = [0          95      0       -90;
                        -90      0       100     0;
                        90       0       100     0];
            for i = 1 : height(joints)
                dhTables(i, 1) = dhTables(i,1) + joints(i);
            end
            endPos = self.dh2fk(dhTables(1:height(joints), :));
        end
        
        % Perform a command cycle. This function will take in a command ID
        % and a list of 32 bit floating point numbers and pass them over the
        % HID interface to the device, it will take the response and parse
        % them back into a list of 32 bit floating point numbers as well
        function com = command(self, idOfCommand, values)
                com= zeros(15, 1, 'single');
                try
                    ds = javaArray('java.lang.Double',length(values));
                    for i=1:length(values)
                        ds(i)= java.lang.Double(values(i));
                    end
                    % Default packet size for HID
                    intid = java.lang.Integer(idOfCommand);
                    self.myHIDSimplePacketComs.writeFloats(intid,  ds);
                    ret = 	self.myHIDSimplePacketComs.readFloats(intid) ;
                    for i=1:length(com)
                       com(i)= ret(i).floatValue();
                    end
                catch exception
                    getReport(exception)
                    disp('Command error, reading too fast');
                end
        end
        
        % This function reads packets recieved from the robot arm
        function com = read(self, idOfCommand)
                com= zeros(15, 1, 'single');
                try

                    % Default packet size for HID
                    intid = java.lang.Integer(idOfCommand);
                    ret = 	self.myHIDSimplePacketComs.readFloats(intid) ;
                    for i=1:length(com)
                       com(i)= ret(i).floatValue();
                    end
                catch exception
                  getReport(exception)
                    disp('Command error, reading too fast');
                end
        end
        
        % This function reads packets recieved from the robot arm
        function  write(self, idOfCommand, values)
                try
                    ds = javaArray('java.lang.Double',length(values));
                    for i=1:length(values)
                        ds(i)= java.lang.Double(values(i));
                    end
                    % Default packet size for HID
                    intid = java.lang.Integer(idOfCommand);
                    self.myHIDSimplePacketComs.writeFloats(intid,  ds,self.pol);

                catch exception
                    getReport(exception)
                    disp('Command error, reading too fast');
                end
        end
        
        % Specifies a position to the gripper
        function writeGripper(self, value)
            try
                ds = javaArray('java.lang.Byte',length(1));
                ds(1)= java.lang.Byte(value);
                intid = java.lang.Integer(self.GRIPPER_ID);
                self.myHIDSimplePacketComs.writeBytes(intid, ds, self.pol);
            catch exception
                getReport(exception)
                disp('Command error, reading too fast');
            end
        end
        
        % Opens the gripper
        function openGripper(self)
            self.writeGripper(0);
        end
        
        % Closes the gripper
        function closeGripper(self)
            self.writeGripper(180);
        end


        % Returns the results for the requested data (GETPOS or GETVEL) and
        % sets the rest to zero.
        function arrFinal = measured_js(self, GETPOS, GETVEL)
            posArr = [0 0 0];
            velArr = [0 0 0];

            if GETPOS
                posArrRead = self.read(1910);
                posArr = [posArrRead(3) posArrRead(5) posArrRead(7)];
            end

            if GETVEL
                velArrRead = self.read(1822);
                velArr = [velArrRead(3) velArrRead(6) velArrRead(9)];
            end

            arrFinal = [posArr; velArr];
        end

        % Returns a 1x3 array with the end-of-motion join setpoint
        % positions. If interpolation is used, return current intermediate
        % set point
        function finalArr = setpoint_js(self)
            jointSetPointPos = self.read(1910);
            finalArr = [jointSetPointPos(2) jointSetPointPos(4) jointSetPointPos(6)];
        end
        
        % This function takes in a 1x3 array of joint values in degrees for the servos
        % to go to without interpolation.
        function servo_jp(self, jvals)
            self.goalJS = jvals;
            servo_packet = zeros(5, 1, 'single');
            servo_packet(1) = 0;        
            servo_packet(2) = 0;           %interpolation
            servo_packet(3) = jvals(1);    %joint 1
            servo_packet(4) = jvals(2);    %joint 2
            servo_packet(5) = jvals(3);    %joint 3
%             disp(servo_packet);
            self.write(self.SERVO_ID, servo_packet);
        end

        % This function takes in a 1x3 array of joint values and an interpolation time
        % in ms to get there
        function interpolate_jp(self, jvals, time_ms)
            self.goalJS = jvals;
            servo_packet = zeros(5, 1, 'single');
            servo_packet(1) = time_ms;        %
            servo_packet(2) = 1;     %interpolation
            servo_packet(3) = jvals(1);    %joint 1
            servo_packet(4) = jvals(2);    %joint 2
            servo_packet(5) = jvals(3);    %joint 3
            self.write(self.SERVO_ID, servo_packet);
            %disp(servo_packet);
            pause(1)
        end
        
        % This function returns the end-point positions for the arm
        function endGoal = goal_js(self)
            endGoal = self.goalJS;
        end

        % Calculates the forward velocity kinematics (or forward
        % differential kinematics) using the current joint positions and
        % current joint velocities.
        function p_dot = fdk3001(self, q, q_dot)
            jacob = self.jacob3001(q);
            p_dot = jacob*deg2rad(q_dot);
            pause(0.001);
        end

        % This function updates the robot arm model, returning the velocity
        % data of the end effector
        function velData = update_model(self, model)
            measured_joint_angles = self.measured_js(true, true);
            jointPos = measured_joint_angles(1,:)';
            jointVel = measured_joint_angles(2,:)';
            eeVel = self.fdk3001(jointPos, jointVel);
%             model.plot_arm(jointPos, eeVel)
            % Call to graph the end effector velocities
%             self.graph.graphVel(eeVel);
            velData = eeVel;
        end

        % Takes in the trajectory coefficients, a duration, and a boolean
        % denoting whether or not it was in task space.
        % returns matrix where 
        % 1:3 = calculated xyz
        % 4:6 = calculated vx vy vz
        % 7:9 = calculated ax ay az
        % 10:15 = jacobian velocities vx, vy, vz, wx, wy, wz
        % 16 = time in ms
        % 17:20 = measured position
        function joint_data = run_trajectory(self, trajectory_coefficients, t, task_space, model)
            j = 1;
            k = zeros([height(trajectory_coefficients) 20]);
            if size (trajectory_coefficients, 2) == 4
                tic;
                while toc < t
                    for i = 1:3
                        a0 = trajectory_coefficients(i, 1);
                        a1 = trajectory_coefficients(i, 2);
                        a2 = trajectory_coefficients(i, 3);
                        a3 = trajectory_coefficients(i, 4);
                        
                        k(j,i) = a0 + a1*toc + a2*toc^2 + a3*toc^3;
                        k(j,i + 3) = a1 + 2*a2*toc + 3*a3*toc^2;
                        k(j, i + 6) = 2*a2 + 6*a3*toc;
                        k(j, 16) = toc;
                    end
                    if task_space
                        k(j, 1:3) = self.ik3001(k(j,1:3));
                    end
                    self.servo_jp(k(j, 1:3));
                    pos = self.fk3001(k(j, 1:3)');
                    k(j, 1:3) = pos(1:3, 4)';
                    k(j, 10:15) = self.update_model(model);
                    measured_joint_angles = self.measured_js(true, true);
                    jacob = self.jacob3001(measured_joint_angles(1,:)');
                    k(j, 17) = det(jacob(1:3,:));
                    k(j, 18:20) = measured_joint_angles(1,:)';
                    j = j + 1;
                end
            elseif size(trajectory_coefficients, 2) == 6
                tic;
                while toc < t
                    for i = 1:3
                        a0 = trajectory_coefficients(i, 1);
                        a1 = trajectory_coefficients(i, 2);
                        a2 = trajectory_coefficients(i, 3);
                        a3 = trajectory_coefficients(i, 4);
                        a4 = trajectory_coefficients(i, 5);
                        a5 = trajectory_coefficients(i, 6);

                        k(j,i) =  a0 + a1*toc + a2*toc^2 + a3*toc^3 + a4*toc^4 + a5*toc^5;
                        k(j,i + 3) = a1 + 2*a2*toc + 3*a3*toc^2 + 4*a4*toc^3 + 5*a5*toc^4;
                        k(j, i + 6) = 2*a2 + 6*a3*toc + 12*a4*toc^2 + 20*a5*toc^3;
                        k(j, 16) = toc;
                    end
                    if task_space
                        k(j, 1:3) = self.ik3001(k(j,1:3));
                    end
                    self.servo_jp(k(j, 1:3));
                    pos = self.fk3001(k(j, 1:3)');
                    k(j, 1:3) = pos(1:3, 4)';
                    k(j, 10:15) = self.update_model(model);
                    measured_joint_angles = self.measured_js(true, true);
                    jacob = self.jacob3001(measured_joint_angles(1,:)');
                    k(j, 17) = det(jacob(1:3,:));
                    k(j, 18:20) = measured_joint_angles(1,:)';
                    j = j + 1;
                end
            end            
            joint_data = k;
            toc
        end
        
        % Creates a unit vector from two 3x1 matrices
        % returns 3x1 matrix representing the unit vector
        function v = unitVector(self, p1, p2)
            dx = p2(1) - p1(1);
            dy = p2(2) - p1(2);
            dz = p2(3) - p1(3);

            magnitude = sqrt(dx^2 + dy^2 + dz^2);
            v = [dx/magnitude; dy/magnitude; dz/magnitude];
        end
    
        % Collects all of the data of the robot arm for plotting
        function data = iVel(self, speed, p1, p2)
            self.unitVector(p1, p2);
            velVector = speed * self.unitVector(p1, p2);

            vx = velVector(1);
            vy = velVector(2);
            vz = velVector(3);

            prevV = [0 0 0];
            prevTime = 0;
            
            t = (p2(1) - p1(1))/vx

            d = zeros([10000 10]);
            tic
            j = 1;

            while toc < t
                x = p1(1)+vx*toc;
                y = p1(2)+vy*toc;
                z = p1(3)+vz*toc;

                joints = self.ik3001([x;y;z]);
                self.servo_jp([joints(1) joints(2) joints(3)]);

                readVals = self.measured_js(true, true);
                pos = self.fk3001(readVals(1, :)');
                pos = pos(1:3, 4);
                readVels = self.fdk3001(readVals(1, :)', readVals(2, :)')';
                readVels = readVels(1, 1:3);

                d(j, 1:3) = pos;
                d(j, 4:6) = readVels;
                d(j, 7:9) = (prevV - readVels) / (toc - prevTime);
                d(j, 10) = toc;
                
                if(d(j, 7) ~= 0 && toc - prevTime > 100)
                    prevTime = toc;
                end
                prevV = readVels;
                j = j + 1;
            end
            toc
            
            d(any(d == 0, 2), :) = [];
            data = d;
        end
        
        % Instructs the robot to grab a ball, given image and trajecotry
        % parameters. A quintic trajectory is implemented here
        function grab_ball(self, time, ts, processor, cam, trajPlanner, m, mask, imgHSV, size)
            camera_height = 180;

            ball_centroids = processor.find_centroids(mask, size);
            ball_height = 10; %processor.find_ball_height(mask, size);

            intrinsic = cam.getCameraInstrinsics();
            pose = cam.getCameraPose();
            rot_mtx = pose(1:3,1:3);
            trans_vector = pose(1:3, 4);
            carry_height = 100;
            
            ball_coords_cf = pointsToWorld(intrinsic, rot_mtx, trans_vector, ball_centroids);

            if height(ball_coords_cf) > 0
                ball_x = ball_coords_cf(1, 1);%*(camera_height - ball_height)/camera_height;
                ball_y = ball_coords_cf(1, 2);%*(camera_height - ball_height)/camera_height;
                ball_z = ball_height;

                %robot frame to checker frame
                rf_to_cf = [0 1 0 80;
                            1 0 0 -100;
                            0  0 1 0;
                            0  0 0  1];
                base_point = rf_to_cf * [ball_x ball_y ball_z 1]';
                
                % Go to ball
                if(ts)
                    robot_pose = self.fk3001(self.measured_js(true, false)');
                else
                    robot_pose = self.measured_js();
                end
    
                vertex1_x_coefficients = trajPlanner.quintic_traj(0, time, 0, 0, 0, 0, robot_pose(1,4), base_point(1))';
                vertex1_y_coefficients = trajPlanner.quintic_traj(0, time, 0, 0, 0, 0, robot_pose(2,4), base_point(2))';
                vertex1_z_coefficients = trajPlanner.quintic_traj(0, time, 0, 0, 0, 0, carry_height, base_point(3) + 30)';
    
                coefficients = [vertex1_x_coefficients; vertex1_y_coefficients; vertex1_z_coefficients];
                
                self.openGripper();
                self.run_trajectory(coefficients, time, ts, m);
                self.interpolate_jp(self.ik3001([base_point(1); base_point(2); base_point(3)])', 500);
                self.closeGripper();
    
                % Go to color bin
                if(ts)
                    robot_pose = self.fk3001(self.measured_js(true, false)');
                else
                    robot_pose = self.measured_js();
                end
                pause(0.5);
                self.interpolate_jp(self.ik3001([robot_pose(1, 4); robot_pose(2, 4); carry_height])', 1000);
    
                drop_off = processor.ball_color(imgHSV, ball_centroids(1, :));
                
                drop_off_x_coef = trajPlanner.quintic_traj(0, time, 0, 0, 0, 0, robot_pose(1,4), drop_off(1))';
                drop_off_y_coef = trajPlanner.quintic_traj(0, time, 0, 0, 0, 0, robot_pose(2,4), drop_off(2))';
                drop_off_z_coef = trajPlanner.quintic_traj(0, time, 0, 0, 0, 0, carry_height, drop_off(3))';
                drop_off_coefs = [drop_off_x_coef; drop_off_y_coef; drop_off_z_coef];
    
                self.run_trajectory(drop_off_coefs, time, ts, m);
                self.openGripper();
    
                if(ts)
                    robot_pose = self.fk3001(self.measured_js(true, false)');
                else
                    robot_pose = self.measured_js();
                end
                self.interpolate_jp(self.ik3001([robot_pose(1, 4); robot_pose(2, 4); carry_height])', 1000);
                pause(0.5);
            end
        end

        % Extra Credit #1: Live tracking ball. Robot arm follows ball as it
        % is moved around checkerboard.
        function point_to_ball(self, processor, cam, mask, size)
            ball_centroids = processor.find_centroids(mask, size);
            
            intrinsic = cam.getCameraInstrinsics();
            pose = cam.getCameraPose();
            rot_mtx = pose(1:3,1:3);
            trans_vector = pose(1:3, 4);

            ball_height = 10;
            
            ball_coords_cf = pointsToWorld(intrinsic, rot_mtx, trans_vector, ball_centroids);

            if height(ball_coords_cf) > 0
                ball_x = ball_coords_cf(1, 1);
                ball_y = ball_coords_cf(1, 2);
                ball_z = ball_height;

                %robot frame to checker frame
                rf_to_cf = [0 1 0 80;
                            1 0 0 -100;
                            0  0 1 0;
                            0  0 0  1];
                base_point = rf_to_cf * [ball_x ball_y ball_z 1]';
                angles = self.ik3001([base_point(1); base_point(2); 150]);
                self.servo_jp(angles);
            end
        end
    end
end
