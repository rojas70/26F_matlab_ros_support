%% Solutions HW01 2025S

%% 01 Compute an SO(2) Rotation

% Set the -50 degree rotation in variable R1
R1 = rot2(-50,'deg');

%% 02 Compute an SE(2) Homogeneous transform

% Compute an SE(2) homogenous transformation matrix that corresponds to a 
% translation of 4 in the y-direction and 3 in the x-direction. 
% Convert the result to a double matrix and place the result in the 
% workspace variable T1. 
% Hint: this can also be done with another 

% In T1 create a homogeneous translation matrix with a translations of 3 and 4 in the x and y directions respectively.
T1 = transl2(3,4)

%% 03 Compute an SE(2) Homogeneous transform

% T2 is a homogeneous transform with a translation of (5,6) in the x- and y-directions and a rotation of -30 degs.
% Note, there are multiple ways to arrive to the final answer, in this case do not use SE2 for now. 
% Compute the final transform via two individual operations.
T2=transl2(5,6)*trot2(-30,'deg')

%% 04 Coordinate Transformation

% Consider a camera pose frame wrt the world as:
%     W_T_C =  transl2(0,1)*trot2(-30, 'deg');
% There is a target object P, whose homogeneous coordinates in the camera coordinate frame  C_P is given by: 
%     C_P = [1,0,1]';
% Determine the coordinate of the target object with respect to the world: W_P


Note:
% Do a hand sketch of both frames and the target object. Make sure you have a good intuition of what is happening here as you will need this a lot in the future. You can also play offline with other values to simplify calculations.

% Camera pose wrt to world
W_T_C = trot2(-30, 'deg') * transl2(0,1);

% Target object homogeneous coordinates wrt 'Camera' frameThere is a target object A, whose homogeneous coordinates in the camera coordinate frame  C_P is given by: 
C_P = [1;0; 1]

% Compute the target object coordinatres wrt to the world
W_P = W_T_C * C_P

%% 05 Coordinate Transforms for a Navigating Robot

% Three variables are defined in the script below.
%   The position of the robot in the world frame, W_R
%   The rotation of the robot with respect to the world frame, theta, and
%   The object coordinates in the world frame, W_P
% Note that all three variables are randomized so that they will differ each time you run the code. 
%
% Your task is to complete the script by:
% Create a homogenous transformation matrix, W_T_R that represents the translation and rotation of the robot with respect to the world frame.
% Use W_T_R to find the object postion in the coordinate frame of the robot. Store the 2D coordinate in R_P (not the homogeneous coordinate)

W_R = randi([-5 5], [2 1])     % Position of robot in the world frame in range [-5 5]
theta = 2*pi*rand              % Rotation of robot in radians
W_P = randi([-5 5], [2 1])     % Position of object in the world frame in range [-5 5]

% Calculate W_T_R and R_P

% Calculate W_T_R
W_T_R = transl2(W_R)*trot2(theta);

% Use the inverse of T (or backslash operator to find the homogenous 
% position in the robot frame. Make sure to add a 1 to destWorld
R_P = inv(W_T_R)*[W_P; 1];

% Now take only the first two elements of pr since we are in 2-D
R_P = R_P(1:2);

%% 06 Euclidean Distance between Two Points (Rigid Body Transformations)
% 
% Consider two objects of interest. 
%
%   Object P_a has coordinates x=1.2 y=-2.7 wrt a coordinate frame A.
%   Object P_b has coordinates x=6.5 y=4.3 wrt a coordinate frame B.
% The transformation from coordinate frame B wrt coordinate frame A is given by the SE2 object:
% A_T_B = SE2(1,2,30,'deg')
% 
% Compute the Euclidean distance between object A and B wrt to coordinate frame A given that you know the pose of B wrt A and place it in variable dis.

% Object P_a coordinates wrt to A
A_P_a = [1.2; -2.7];

% Object P_b coordinates wrt to B
B_P_b = [6.5; 4.3];

% Pose of B wrt of A will be created as an SE2 Object
A_T_B = SE2(1,2,30,'deg');

% Compute dis
 
% Convert B_P_b coordinates to homogeneous coordinates
B_P_b = [B_P_b; 1];

% Transform coordinates to A
A_P_b = A_T_B.T * B_P_b;

dis = norm(A_P_a - A_P_b(1:2)); 