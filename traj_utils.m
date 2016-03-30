classdef traj_utils < handle
    properties
    end
    
    methods (Static)
        function [traj] = joint_traj(init_joint_state, final_joint_state, steps)
            [m1, n1] = size(init_joint_state);
            [m2, n2] = size(final_joint_state);
            assert(m1==1 && m2==1, 'Need to pass in a row vector with joint state');
            assert(n1==n2, 'Number of initial and final joint states must match');
            num_joints = n1;
            
            step_size = zeros(1, num_joints);
            for i=1:num_joints
                step_size(i) = (final_joint_state(i) - init_joint_state(i))/steps;
            end
            traj = zeros(steps, num_joints);
            for i=1:steps
                for j=1:num_joints
                 traj(i, j) = init_joint_state(j)+step_size(j)*i;
                end 
            end
        end
    end
    
end

