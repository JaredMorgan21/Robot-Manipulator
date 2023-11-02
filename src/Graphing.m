classdef Graphing
    %Graphing Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        linVel = zeros();
        angVel = zeros();
        scaleVel = zeros();
    end
    
    methods
        function self = Graphing()
           
        end

        %Generates the graph from the given data
        %Admittedly, still need to deduce how to handle things like adding
        %a legend
        function graph2D(self, fig_num, x_axis, y_axis, x_title, y_title)
            hold on
            plot(x_axis, y_axis);
            xtitle(x_title);
            ytitle(y_title);
            hold off
            view(2);
            figure(fig_num);
        end
        
        




    end
end

