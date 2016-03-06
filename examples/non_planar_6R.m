% Non-Planar RRR Arm Example

clear;
close all;

global DebugLevel;
DebugLevel = 1;

% Specify the link twist co-ordinates.
% For revolute joint we use the following form:
% [-w1 x q1, -w2 x q2, -w3 x q3, w1, w2, w3]

% 6-link non-planar RRR arm
lengthA = 1;
lengthB = 1;
lengthC = 1;
lengthD = 1;
lengthE = 1;
lengthF = 1;
lengthG = 1;

joint1 = [0; 0; 0; 0; 0; 1];
joint2 = [-(lengthA + lengthB); 0; 0; 0; 1; 0];
joint3 = [0; 0; 0; 0; 0; 1];
joint4 = [-(lengthA + lengthB + lengthC + lengthD); 0; 0; 0; 1; 0];
joint5 = [0; 0; 0; 0; 0; 1];
joint6 = [-(lengthA + lengthB + lengthC + lengthD + lengthE + lengthF); 0; 0; 0; 1; 0];

% Specify the end-effector reference/home pose
% as a homogeneous matrix.
% E.g. the end-effector is at [x,y,z] = [0, 2, 0]
% with no rotation.
M = eye(4);
M(3,4) = (lengthA + lengthB + lengthC + lengthD + lengthE + lengthF + lengthG);

% Create the robot
RRR = robot({joint1, joint2, joint3, joint4, joint5, joint6}, M);

% Create a trajectory in joint space, only moving joint 2.
q1 = [-pi/2:pi/16:pi/2];
q2 = [-pi/2:pi/16:pi/2];
%q1 = zeros(size(q2));
q3 = zeros(size(q2));
q4 = zeros(size(q2));
q5 = zeros(size(q2));
q6 = zeros(size(q2));
joint_space_trajectory = [q1; q2; q3; q4; q5; q6]

% Convert the joint-space trajectory into the end-effector
% trajectory using the forward-kinematics.
end_effector_trajectory = fkine(RRR, joint_space_trajectory);

% Plot the results
named_figure('Cartesian Trajectory of the End-Effector');
drawframetraj(end_effector_trajectory);

% Compute the body Jacobian at reference pose (vertical)
body_jacobian = bjacob(RRR, [0; 0; 0; 0; 0; 0])

% Calculate the instantaneous end-effector body velocity for a
% given set of joint velocities
body_velocity = body_jacobian * [0; 0.1; 0; 0; 0; 0]

% Evalute the spatial Jacobian also at the reference pose. 
spatial_jacobian = sjacob(RRR, [0; 0; 0; 0; 0; 0])

% Calculate the instantaneous end-effector spatial velocity for a
% given set of joint velocities
spatial_velocity = spatial_jacobian * [0; 0.1; 0; 0; 0; 0]
