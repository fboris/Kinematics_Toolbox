% Inverse Differential Kinematics (IDK) Example Using RRR Planar Arm 

clear;
close all;

global DebugLevel;
DebugLevel = 1;

% Specify the link twist co-ordinates.
% For revolute joint we use the following form:
% [-w1 x q1, -w2 x q2, -w3 x q3, w1, w2, w3]

% 3-link planar RRR arm
lengthA = 2;
lengthB = 3;
lengthC = 1;

joint1 = [0; 0; 0; 0; 0; 1];
joint2 = [lengthA; 0; 0; 0; 0; 1];
joint3 = [lengthB; 0; 0; 0; 0; 1];

% Specify the end-effector reference/home pose
% as a homogeneous matrix.
% E.g. the end-effector is at [x,y,z] = [0, 2, 0]
% with no rotation.
M = eye(4);
M(1, 4) = 2;

% Create the robot
RRR = robot({joint1, joint2, joint3}, M);

% Compute the IDK for a constant end-effectory velocity of v_x = 0.05m/s for 5s.
% I.e. dx = 25cm. We expect the end-effector to move from (2, 0, 0) to (2.25, 0, 0).
n_trajectory_entries = 100;
dt = 0.05;
desired_end_effector_body_velocity = [0.05; 0; 0];
joint_angles = zeros(3, n_trajectory_entries);

% Fill the trajectory.
for i=1:n_trajectory_entries
  % Evaluate the body jacobian at the current joint angles.
  body_jacobian = bjacob(RRR, joint_angles(:,i));
  inv_body_jacobian = pinv(body_jacobian);

  % Compute the joint velocities that will achieve the desired end-effector body velocity.
  joint_velocities = inv_body_jacobian(:, 1:3) * desired_end_effector_body_velocity;

  % Compute the next trajectory point using simple integration.
  joint_angles(:, i+1) = joint_angles(:, i) + (dt * joint_velocities);
end

% For display purposes convert the joint-space trajectory into an end-effector trajectory
end_effector_trajectory = fkine(RRR, joint_angles);

% Plot the end-effector trajectory
named_figure('Cartesian Trajectory of the End-Effector');
drawframetraj(end_effector_trajectory);
