# Experiments
for matlabs
%% 1) PARAMETERS
g = 9.81;                 % gravity (m/s^2)
L = 0.60;                 % beam length (m)
beamWidth = 0.065;        % beam width (m)
beamThickness = 0.025;    % beam thickness (m)
R = 0.020;                % ball radius (m)

xTarget = 0.0;            % desired ball position (m)
xInitial = 0.20;          % initial ball position (m)
vInitial = 0.0;           % initial ball velocity (m/s)
thetaInitial = 0.0;       % initial beam angle (rad)

% PD controller
Kp = 1.8;
Kd = 1.15;
thetaMax = deg2rad(12);

% Beam actuator (first-order response)
actuatorTimeConstant = 0.08;  % seconds

% Rolling resistance / damping
velocityDamping = 0.12;       % 1/s

% Simulation and animation
tEnd = 8;                     % seconds
animationFrames = 260;
baseLength = 0.52;
baseWidth = 0.22;
baseThickness = 0.04;
pivotHeight = 0.38;
bearingHeight = 0.035;
columnLength = pivotHeight ...
    - beamThickness/2 ...
    - bearingHeight ...
    - baseThickness;
columnWidth = 0.065;
columnDepth = 0.075;
bearingHeight = 0.035;
