classdef tf
    %UNTITLED4 Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
    end
    
    methods(Static)
        function [T] = rotz_t(theta)
%             T = [cos(theta),    -sin(theta),    0,  0; ...
%                  sin(theta),    cos(theta),     0,  0; ...
%                  0,             0,              1,  0; ...
%                  0,             0,              0,  1];
            T = makehgtform('zrotate', theta);
        end
        
        function[T] = rotx_t(theta)
%             T = [1,     0,          0,           0; ...
%                  0,     cos(theta), -sin(theta), 0; ...
%                  0,     sin(theta),  cos(theta), 0; ...
%                  0,     0,          0,           1];
             T = makehgtform('xrotate', theta);
        end
        
        function [T] = roty_t(theta)
%             T = [cos(theta),    0,     sin(theta),  0; ...
%                  0,             1,     0,           0; ...
%                  -sin(theta),   0,     cos(theta),  0; ...
%                  0,             0,     0,           1];
             T = makehgtform('yrotate', theta);
        end
        
        function [T] = translate(x, y, z)
%             T = [1,    0,     0,  x; ...
%                  0,    1,     0,  y; ...
%                  0,    0,     1,  z; ...
%                  0,    0,     0,  1];
            T = makehgtform('translate', [x, y, z]);
        end

        
    end
    
end

