classdef RobotController_3DOF < handle
    %UNTITLED Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        arduino_handle
        joint_servos
        other_servos
    end
    methods (Access = private)
        function [servos] = servos_initialize(obj, minPulseDuration, maxPulseDuration, num_servos, pin_list)
            for i=1:num_servos
                servos(i) = servo(obj.arduino_handle, strcat('D', int2str(pin_list(i))), ...
                    'MinPulseDuration', minPulseDuration ...
                    ,'MaxPulseDuration', maxPulseDuration);
            end
        end
    end
    methods (Access = public)
        function robotController = RobotController_3DOF(pulse_duration, arduino_pin_list)
            assert(length(arduino_pin_list)==3, 'Program currently only supports 3 servo robots');
            assert(length(pulse_duration)==2, 'Pulse duration includes one min and one max. Length should be 2');
            robotController.arduino_handle = arduino();
            robotController.joint_servos = robotController.servos_initialize(pulse_duration(1), pulse_duration(2), length(arduino_pin_list), arduino_pin_list);
        end
        function [] = go_home(obj)
            for i=1:length(obj.joint_servos)
                writePosition(obj.joint_servos(i), 0.0);
            end
        end
        function [] = sweep_demo(obj)
            theta = 0;
            delta = 1;
            while theta>-1
                for i=1:length(obj.joint_servos)
                    writePosition(obj.joint_servos(i), theta/180);
                end
                theta = theta + delta;
                if theta==180
                    delta = -1;
                elseif theta == 0
                    delta = 1;
                end
            end
        end
    end
    
end

