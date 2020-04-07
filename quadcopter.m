  % X config
clear
clc
%
% Loading the Q Matrix, kP and kD gain vectors used during Training
Q_file = load('Q_matrices.mat','Q');
kP_file = load('kP_vectors.mat','kP');
kD_file = load('kD_vectors.mat','kD');

mass = 0.25; % mass of quadcopter
iX = 0.03; % kgm^2
iY = 0.03; % kgm^2
iZ = 0.03; % kgm^2
% thrustCoeff = 0.1;
% rho = 1.225;
% rotorRadius = 0.035;
% rotorArea = pi*rotorRadius^2;
% thrustFactor = thrustCoeff*rho*rotorArea*rotorRadius^2;
thrustFactor = 0.1;
dragFactor = 0.1;
armLength = 0.1;
%% Initial conditions
%
x0 = 0.0;
y0 = 0.0;
z0 = 0.00000001;
xDot0 = 0.0;
yDot0 = 0.0;
zDot0 = 0.00000001;
phi0 = 0.0;
theta0 = 0.0;
psi0 = 0.0;
phiDot0 = 0.0;
thetaDot0 = 0.0;
psiDot0 = 0.0;
%
%% Input (used for regular flight without any trajectory)
% The inputs are propeller speeds and are given such that the quadcopter
% pitches forward and move along x-axis while climbing up.
%om1 = 2.8;
%om2 = 3.2;
%om3 = 3.2;
%om4 = 2.8;
%
%% Trajectory generation
%
xIni = 0.0;
xFinal = 1.0;
yIni = 0.0;
yFinal = 1.0;
zIni = 0.0;
zFinal = 1.0;
%
Ts = 0.1;
totTime = 2.5;
%
t = 0.0:Ts:totTime;
%
xGoal = linspace(xIni,xFinal,length(t));

yGoal = linspace(yIni,yFinal,length(t));

zGoal = linspace(zIni,zFinal,length(t));

% Trajectory for a Diagonal of cube with length 1 unit
trajectory = zeros(length(t),4);

trajectory(:,1) = t';
%
for i = 1:length(t)
    trajectory(i,2:end) = [xGoal(i) yGoal(i) zGoal(i)];
    h = animatedline(xGoal(i),yGoal(i),zGoal(i));
end

Q = Q_file.Q(:,:,:)*100;
kP = kP_file.kP(:,:);
kD = kD_file.kD(:,:);


% PID Simulation
simout = sim('quadcopter_control_2018a','OutputOption', 'SpecifiedOutputTimes', ...
    'OutputTimes', '0:Ts:totTime');
% Saving the Trajectory data for Training
gain = simout.Gain.Data(:,:);
data = simout.simout.Data(:,:);
save('quadcopter_cube_diag','data');
% Tracking the path of the Quadcopter
laser_tracking = simout.get('laser_tracking');
pos_actual = simout.get('pos_actual');
