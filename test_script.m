clear
clc

pulse_duration = [600*10^-6  2450*10^-6];
pin_numbers = [9, 10, 11];

robot_controller = RobotController_3DOF(pulse_duration, pin_numbers);
pause(2.0);
robot_controller.go_home();
pause(2.0);
robot_controller.sweep_demo();
clear