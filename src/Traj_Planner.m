classdef Traj_Planner
    %TRAJ_PLANNER Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        robot;
    end
    
    methods
        
        % Returns a 4x1 array of trajectory coefficients
        % Takes in a durations, and start and end positions and velocities
        function A = cubic_traj(self, t, startVel, endVel, startPos, endPos)
            coeff_mtx = [ 1 0   0       0;
                          0 1   0       0;
                          1 t   t^2   t^3;
                          0 1   2*t     3*t^2];

            bounds = [startPos; startVel; endPos; endVel];
            A = inv(coeff_mtx) * bounds;
        end
        
        % Returns a 6x1 array of trajectory coefficients
        % Takes in a start and end time, position, velocity, and
        % acceleration
        function A = quintic_traj(self, t0, tf, v0, vf, a0, af, p0, pf)
            coeff_mtx = [1 t0   t0^2    t0^3    t0^4    t0^5;
                         0 1    2*t0    3*t0^2  4*t0^3  5*t0^4;
                         0 0    2       6*t0    12*t0^2 20*t0^3;
                         1 tf   tf^2    tf^3    tf^4    tf^5;
                         0 1    2*tf    3*tf^2  4*tf^3  5*tf^4;
                         0 0    2       6*tf    12*tf^2 20*tf^3;];

            bounds = [p0; v0; a0; pf; vf; af];

            A = inv(coeff_mtx) * bounds;
        end
    end
end

