% Numerical code for paper titled: 
% "A geometric approach towards inverse kinematics of soft extensible 
%  pneumatic actuators intended for trajectory tracking".
% https://arxiv.org/pdf/2211.06259.pdf
%
% version: v01.01.01 230608
%
% Author: Arman Goshtasbi (argo@mmmi.sdu.dk) 
%
%
% This code generates the state variables of a 3d multi-segment extensible
% soft-robot on any desired trajectory. 
% In order to run the code, first define robot configurations and also the
% desired path. 
% The output of the code is:  L (length of each soft segment), 
% phi (deflection angle of each segment with respect to the X-axis) , 
% theta (bending angle of each soft segment)




%% Defining the Robot Configuration and Path
clear,clc,close all

%Robot Configurations
numseg = 3; %number of segments in the robot
num_link = 6; %number of link in each segments
P_s = [0,0,0]; % the base position of the robot
L = 65; % length of each segment without inflation
P_initial = P_s + L*numseg*[0,0,1]; % Initial end effector position
L_link = L/num_link; % The length of each link before inflation

% The Desired Path 
t1 = linspace(-pi/2,pi/2,250);
Px1 = 100+30*cos(t1);
Pz1 =100+30*sin(t1).*cos(t1);
Py1 = 100-15*abs(sin(t1+pi/2));
t2 = linspace(pi/2,3*pi/2,250);
Px2 = 100+30*cos(t2).*sin(t2);
Py2 =100+30*cos(t2);
Pz2 = 100+15*abs(sin(t2+pi/2));
t3 = linspace(3*pi/2,5*pi/2,250);
Px3 = 100+30*cos(t3).*sin(t3);
Py3 =100+30*cos(t3);
Pz3 = 100-15*abs(sin(t3+pi/2));
t4 = linspace(5*pi/2,7*pi/2,250);
Px4 = 100+30*cos(t4);
Pz4 =100+30*sin(t4).*cos(t4);
Py4 = 100+15*abs(sin(t4+pi/2));
Px = [Px1,Px2,Px3,Px4];
Py = [Py1,Py2,Py3,Py4];
Pz = [Pz1,Pz2,Pz3,Pz4];
P_d = [Px;Py;Pz];

%% solving The Inverse Kinematics
Solution = Inverse_Kinematics_final(P_d,L_link,numseg,num_link,false);

%% The calculated parameters
theta = Solution.th;
phi = Solution.phi;
L = Solution.L;





