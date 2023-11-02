classdef Model < handle
    properties
        robot;
        armPlot = plot3(0, 0, 0,'-o','LineWidth',2,'MarkerSize',6);
        frame1;
        frame2;
        frame3;
        velQuiver;
    end

    methods
        
        % Create a packet processor for an HID device with USB PID 0x007
        function self = Model(bot)
            self.robot = bot;
            grid on;
            title("3D Arm Model");
            xlabel("x (mm)");
            ylabel("y (mm)");
            zlabel("z (mm)");
            axis([-400 400 -400 400 0 400]);
        end

        % Takes in an mxn matrix of measured joint values
        function plot_arm(self, joints, jointVelocities)
            pause(0.0001);
            delete(self.frame1);
            delete(self.frame2);
            delete(self.frame3);
            delete(self.velQuiver);

            T01 = self.robot.fk3001(joints(1));
            T02 = self.robot.fk3001(joints(1:2));
            T03 = self.robot.fk3001(joints(1:3));
            
            xVals = [0 T01(1, 4) T02(1, 4) T03(1, 4)];
            yVals = [0 T01(2, 4) T02(2, 4) T03(2, 4)];
            zVals = [0 T01(3, 4) T02(3, 4) T03(3, 4)];

            xVel = jointVelocities(1);
            yVel = jointVelocities(2);
            zVel = jointVelocities(3);

            hold on
            set(self.armPlot,"XData", xVals,"YData", yVals, "ZData",zVals, 'Color', 'black')
            % Coordinate Frames
            quiver3(0, 0, 0, 50, 0, 0, "red", "LineWidth", 3);
            quiver3(0, 0, 0, 0, 50, 0, "blue", "LineWidth", 3);
            quiver3(0, 0, 0, 0, 0, 50, "green", "LineWidth", 3);
            self.frame1 = self.drawQuiver(T01);
            self.frame2 = self.drawQuiver(T02);
            self.frame3 = self.drawQuiver(T03);
            self.velQuiver = quiver3(T03(1,4), T03(2,4), T03(3,4), xVel, yVel, zVel, "magenta", "LineWidth", 3);
            hold off
        end

        % Generates the quivers for the coordinate frams of the joints of
        % the robot arm
        function frame = drawQuiver(self, transMatrix)
            x = quiver(NaN, NaN, NaN, NaN);
            y = quiver(NaN, NaN, NaN, NaN);
            z = quiver(NaN, NaN, NaN, NaN);
            
            delete(x)
            delete(y)
            delete(z)

            xDir = transMatrix(1:3, 1:3) * [50; 0; 0];
            yDir = transMatrix(1:3, 1:3) * [0; 50; 0];
            zDir = transMatrix(1:3, 1:3) * [0; 0; 50];

            % X, Y, Z
            x = quiver3(transMatrix(1, 4), transMatrix(2, 4), transMatrix(3, 4), xDir(1), xDir(2), xDir(3), "red", "LineWidth", 3);
            y = quiver3(transMatrix(1, 4), transMatrix(2, 4), transMatrix(3, 4), yDir(1), yDir(2), yDir(3), "blue", "LineWidth", 3);
            z = quiver3(transMatrix(1, 4), transMatrix(2, 4), transMatrix(3, 4), zDir(1), zDir(2), zDir(3), "green", "LineWidth", 3);
            
            frame = [x y z];
        end
    end
end