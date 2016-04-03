function [] = runGrassy()
    clear;
    clear a servos;
    
    L(1) = LinkDef([0,     69,     0,     pi/2,    0]);
    L(2) = LinkDef([0,     22,     85,   -pi,      0]);
    L(3) = LinkDef([0,     22,     60,    0,       0]);
    
    robot = RobotDef(L);
    
    
%%  Start the link plot
    grassy = create_grassy_serial_link();
    plot_grassy(grassy, 0, 0, 0);
    
    goalState = makehgtform('translate', [0 60.0 50.3002]);
    final_js = robot.invKinematics_3(goalState)
    robot.fwdKinematics([0,0,0], 0)
%%  Initialize arduino object and servos
    a = arduino();
    servos = servo_initialize(a, 600*10^-6, 2450*10^-6, 4, [9, 10, 11, 6]);
%%  Initialize gripper servo to 0
    writePosition(servos(4), 0.0);
    writePosition(servos(1), 0.0);
    writePosition(servos(2), 0.0);
    writePosition(servos(3), 0.0);
    pause(4.0);
%%  pre allocate memory for some variables
    time = 300;
    record = 0;
    t = zeros(1, 2000);
    q = zeros(3, 2000);
    j=0;
    p_input = zeros(3);
%% Joint interpolated path planning test
    init_js = [0, readPosition(servos(1))*180, readPosition(servos(2))*180, readPosition(servos(3))*180]
    
    goalState = makehgtform('translate', [0 60.0 50.3002]);
    final_js = robot.invKinematics_3(goalState)
    final_js_check = grassy.ikine3(goalState)
    steps = 50;
    traj = traj_utils.joint_traj(init_js, final_js*180/pi, steps)
    for i=1:steps
        for j=2:4
            writePosition(servos(j-1), traj(i, j)/180);
        end
        plot_grassy(grassy, traj(i, 2)*pi/180,  traj(i, 3)*pi/180,  traj(i, 4)*pi/180);
%         pause(0.01);
    end
    robot.fwdKinematics(final_js)
    grassy.fkine(final_js)
    
%%  While loop for operating with robot
%     while time>=0
%         record_input = readDigitalPin(a, 'D2');
%         if record_input==1
%             if record==0
%                 record = 1;
%                 display('Recording ...');
%             else
%                 record = 0;
%                 display('... Stopped Recording');
%             end
%         end
%         p_input(1) = readVoltage(a, 'A1');
%         p_input(2) = readVoltage(a, 'A2');
%         p_input(3) = readVoltage(a, 'A3');
%         for i=1:1:3
%             writePosition(servos(i), p_input(i)/5.0);
%         end
%         pause(0.005);
%         if record==1
%             j = j+1;
%             t(j) = j;
%             for i=1:1:3
%                 q(i,j) = readPosition(servos(i))*180;
%             end
%         end
%         time = time-0.1;
%     end
% %%  Plot angles observed if recorded
%     t = t(t~=0);
%     [m, n] = size(t);
% 
%     for i=1:1:3
%         joint_angles = q(i,1:n);
%         plot(t, joint_angles);
%         hold on
%     end
% 
%     hold off
% %%  Replay recorded data
%     pause(2.0)
%     for j=1:1:n
%         writePosition(servos(1), q(1, j)/180);
%         writePosition(servos(2), q(2, j)/180);
%         writePosition(servos(3), q(3, j)/180);
%         pause(0.1);
%     end
    clear servos a
end 

%%
function [] = plot_grassy(serial_link, q1, q2, q3)
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here
    serial_link.plot([q1, q2, q3]);

end

%%
function [serial_link_handle] = create_grassy_serial_link()
    L(1) = Link([0,     65,     0,     pi/2,    0]);
    L(2) = Link([0,     22,     90,   -pi,      0]);
    L(3) = Link([0,     22,     50,    0,       0]);

    serial_link_handle = SerialLink(L, 'name', 'Grassy');    
end

%%
function [ servos ] = servo_initialize(arduino_handle, minPulseDuration, maxPulseDuration, num_servos, pin_list)
%UNTITLED2 Summary of this function goes here
%   Detailed explanation goes here 600*10^-6  2450*10^-6
    for i=1:1:num_servos
        servos(i) = servo(arduino_handle, strcat('D', int2str(pin_list(i))), 'MinPulseDuration', minPulseDuration ...
            , 'MaxPulseDuration', maxPulseDuration);
    end

end
%%