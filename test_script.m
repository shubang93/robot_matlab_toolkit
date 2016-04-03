clear
clc

%% Initial parameters for robot
pulse_duration = [600*10^-6  2450*10^-6];
pin_numbers = [9, 10, 11];
L(1) = LinkDef([0,     69,     0,     pi/2,    0]);
L(2) = LinkDef([0,     22,     85,   -pi,      0]);
L(3) = LinkDef([0,     22,     60,    0,       0]);

%% INitializing arduino robot
robot_controller = RobotController_3DOF(pulse_duration, pin_numbers, L);
pause(2.0);

%% Asking robot to go home first
robot_controller.go_home();

%% Going to a position in joint space movement
goalState = makehgtform('translate', [0 60.0 50.3002]);
robot_controller.move_jointSpace(goalState);

%% Clearing all variables
clear