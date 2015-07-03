% RRP Arm Example

clear;
close all;

global DebugLevel;
DebugLevel = 1;

% Specify the link twist co-ordinates.
% For revolute joint we use the following form:
% [-w1 x q1, -w2 x q2, -w3 x q3, w1, w2, w3]

% Joint 1 is rotating about the z axis.
joint1 = [0; 0; 0; 0; 0; 1];

% Joint 2 is rotating about the x axis offset to (0, 10, 10)
joint2 = [0; 10; -10; 1; 0; 0];

% Joint 3 is prismatic, sliding along the y axis.
joint3 = [0; 1; 0; 0; 0; 0];

% Specify the end-effector reference/home pose
% as a homogeneous matrix.
% E.g. the end-effector is at [x,y,z] = [0, 2, 0]
% with no rotation.
M = eye(4);
M(2,4) = 2;

% Create the robot
RRP = robot({joint1, joint2, joint3}, M);

% Create a trajectory is joint space, moving only joint 2.
q2 = [-pi/2:pi/16:pi/2];
q1 = zeros(size(q2));
q3 = zeros(size(q2));
joint_space_trajectory = [q1; q2; q3];

% Convert the joint-space trajectory into the end-effector
% trajectory using the forward-kinematics.
cartesian_trajectory = fkine(RRP, joint_space_trajectory);

% Plot the results
named_figure('Cartesian Trajectory of the End-Effector');
drawframetraj(cartesian_trajectory);
