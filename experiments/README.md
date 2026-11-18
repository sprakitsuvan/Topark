# Experiments

for matlabs and simulink

%% 1) PARAMETERS

g = 9.81;

% Beam and ball dimensions (meters)
L = 0.150;                 % Beam length = 150 mm
beamWidth = 0.01625;       % 16.25 mm
beamThickness = 0.00625;   % 6.25 mm
R = 0.005;                 % Ball radius = 5 mm

% Initial conditions
xTarget = 0.0;
xInitial = 0.050;          % 50 mm
vInitial = 0.0;
thetaInitial = 0.0;

% PD controller
Kp = 7.2;
Kd = 4.6;
thetaMax = deg2rad(12);

% Beam actuator
actuatorTimeConstant = 0.08;

% Rolling resistance / damping
velocityDamping = 0.12;



for simulink only

% Simulation and animation
tEnd = 8;
animationFrames = 260;

%% 2) STAND DIMENSIONS

baseLength = 0.130;
baseWidth = 0.055;
baseThickness = 0.010;

pivotHeight = 0.095;
bearingHeight = 0.00875;

columnLength = pivotHeight ...
    - beamThickness/2 ...
    - bearingHeight ...
    - baseThickness;

columnWidth = 0.01625;
columnDepth = 0.01875;

bearingLength = 0.02625;
bearingWidth = 0.02875;

shaftRadius = 0.003;
