
# Application software

Document the target PC or Raspberry Pi, operating system, runtime, dependencies, installation, configuration, and run instructions. Keep secrets in local environment variables, not in the repository.
%% BALL AND BEAM BALANCE SYSTEM - MATLAB ONLY (WITH GROUND / STAND)
clc;
clear;
close all;

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

%% 2) RUN THE BALL-AND-BEAM DYNAMICS

y0 = [xInitial; vInitial; thetaInitial];

params.g = g;
params.Kp = Kp;
params.Kd = Kd;
params.thetaMax = thetaMax;
params.tau = actuatorTimeConstant;
params.damping = velocityDamping;
params.xTarget = xTarget;
params.xLimit = L/2 - R;

opts = odeset('RelTol',1e-7,'AbsTol',1e-9);
[t,y] = ode45(@(t,y) ballBeamODE(t,y,params), [0 tEnd], y0, opts);

x = y(:,1);
v = y(:,2);
theta = y(:,3);

thetaCommand = zeros(size(t));
for k = 1:numel(t)
    thetaCommand(k) = controllerAngle(x(k),v(k),params);
end

%% 3) PLOT RESPONSE GRAPHS
figure('Name','Ball and Beam - Response','Color','w');

subplot(3,1,1);
plot(t,x,'LineWidth',1.8);
hold on;
yline(xTarget,'--','Target');
grid on;
ylabel('Ball position x (m)');
title('Ball and Beam Closed-Loop Response');

subplot(3,1,2);
plot(t,rad2deg(theta),'LineWidth',1.8);
hold on;
plot(t,rad2deg(thetaCommand),'--','LineWidth',1.2);
grid on;
ylabel('Beam angle (deg)');
legend('Actual angle','Command angle','Location','best');

subplot(3,1,3);
plot(t,v,'LineWidth',1.8);
grid on;
ylabel('Ball velocity (m/s)');
xlabel('Time (s)');

%% 4) BUILD 3-D RIG: GROUND + BASE + SUPPORT + BEAM + BALL
fig = figure('Name','3-D Ball and Beam Balance System', ...
             'Color',[0.96 0.96 0.96]);
ax = axes('Parent',fig);
hold(ax,'on');
grid(ax,'on');
axis(ax,'equal');
view(ax,[-34 22]);

xlabel(ax,'X (beam direction) [m]');
ylabel(ax,'Y (width) [m]');
zlabel(ax,'Z (height) [m]');
title(ax,'Ball and Beam Balance System with Fixed Ground');

xlim(ax,[-0.43 0.43]);
ylim(ax,[-0.24 0.24]);
zlim(ax,[-0.035 0.56]);

% Fixed ground plane (visual reference)
[gX,gY] = meshgrid(linspace(-0.48,0.48,2), ...
                   linspace(-0.28,0.28,2));
gZ = zeros(size(gX));
surf(ax,gX,gY,gZ, ...
    'FaceColor',[0.82 0.84 0.86], ...
    'FaceAlpha',0.35, ...
    'EdgeColor',[0.68 0.70 0.72], ...
    'LineStyle','-');

% Fixed base plate resting on the ground
makeBox(ax,[baseLength baseWidth baseThickness], ...
    [0 0 baseThickness/2],[1 0 0]);

% Fixed vertical support column
columnCenterZ = baseThickness + columnLength/2;
makeBox(ax,[columnWidth columnDepth columnLength], ...
    [0 0 columnCenterZ],[1 0 0]);

% Bearing block at the top of the support
bearingCenterZ = pivotHeight - beamThickness/2 - bearingHeight/2;
makeBox(ax,[0.105 0.115 bearingHeight], ...
    [0 0 bearingCenterZ],[1 0 0]);

% Pivot shaft, with its axis along Y
a = linspace(0,2*pi,36);
yy = linspace(-beamWidth*0.72,beamWidth*0.72,3);
[A,Ycyl] = meshgrid(a,yy);
shaftRadius = 0.012;
Xcyl = shaftRadius*cos(A);
Zcyl = pivotHeight + shaftRadius*sin(A);
surf(ax,Xcyl,Ycyl,Zcyl, ...
    'FaceColor',[1 ,0 , 0], ...
    'EdgeColor','none');

% Center mark (fixed in X, directly above the pivot)
centerMarker = plot3(ax,0,0,pivotHeight+beamThickness/2+0.003, ...
    'k+','MarkerSize',11,'LineWidth',1.6);

% Beam as a 3-D rectangular solid; vertices will be rotated every frame
[beamVertices,beamFaces] = boxMesh([L beamWidth beamThickness]);
beamWorldVertices = transformBeam(beamVertices,0,pivotHeight);
beamPatch = patch(ax,'Vertices',beamWorldVertices,'Faces',beamFaces, ...
    'FaceColor',[0 1 0], ...
    'EdgeColor',[0.04 0.05 0.07], ...
    'LineWidth',0.8);

% Ball as a 3-D sphere
[sx,sy,sz] = sphere(24);
ballSurf = surf(ax,R*sx,R*sy,R*sz, ...
    'FaceColor',[0 0 1], ...
    'EdgeColor',[0.25 0.27 0.30], ...
    'LineWidth',0.35);

% Labels
text(ax,-baseLength/2+0.015, -baseWidth/2-0.025, baseThickness+0.006, ...
    'GROUND / FIXED BASE','FontSize',8,'FontWeight','bold');
statusText = text(ax,-0.39,-0.20,0.52,'', ...
    'FontSize',9,'FontWeight','bold','Color',[0.12 0.15 0.19]);

% Lighting and camera
camlight(ax,'headlight');
lighting(ax,'gouraud');
material(ax,'dull');
set(ax,'FontSize',9,'LineWidth',0.8);
rotate3d(fig,'on');

%% 5) ANIMATE BEAM AND BALL
frameIndices = unique(round(linspace(1,numel(t),animationFrames)));

for q = 1:numel(frameIndices)
    k = frameIndices(q);
    th = theta(k);

    % Rotate beam about the fixed pivot axis (Y axis)
    Vworld = transformBeam(beamVertices,th,pivotHeight);
    set(beamPatch,'Vertices',Vworld);
    markerX = -beamThickness/2*sin(th);
    markerZ = pivotHeight + beamThickness/2*cos(th) + 0.003;
    set(centerMarker,'XData',markerX,'YData',0,'ZData',markerZ);

    % Ball center is R above the top surface, measured normal to the beam.
    % For positive theta, the beam's right end rises.
    ballLocalX = x(k);
    ballLocalZ = beamThickness/2 + R;
    ballCenterX = ballLocalX*cos(th) - ballLocalZ*sin(th);
    ballCenterZ = pivotHeight + ballLocalX*sin(th) + ballLocalZ*cos(th);

    set(ballSurf, ...
        'XData',R*sx+ballCenterX, ...
        'YData',R*sy, ...
        'ZData',R*sz+ballCenterZ);

    set(statusText,'String',sprintf( ...
        'Time: %.2f s   Ball x: %.3f m   Beam: %.2f deg', ...
        t(k),x(k),rad2deg(th)));

    drawnow;
    pause(0.008);
end

%% 6) PRINT FINAL RESULT
fprintf('\n===== BALL AND BEAM RESULT =====\n');
fprintf('Initial ball position : %.3f m\n',xInitial);
fprintf('Target ball position  : %.3f m\n',xTarget);
fprintf('Final ball position   : %.4f m\n',x(end));
fprintf('Final ball velocity   : %.4f m/s\n',v(end));
fprintf('Final beam angle      : %.3f deg\n',rad2deg(theta(end)));
fprintf('Moving DOF            : 2 (ball translation + beam rotation)\n');
fprintf('Ground / stand DOF    : 0 (fixed)\n');
fprintf('================================\n');

if abs(x(end)-xTarget) < 0.01 && abs(v(end)) < 0.02
    disp('Result: ball is close to the center and nearly stopped.');
else
    disp('Result: try tuning Kp, Kd, or the actuator time constant.');
end

%% LOCAL FUNCTIONS
function dydt = ballBeamODE(~,y,p)
    x = y(1);
    ballVelocity = y(2);
    beamAngle = y(3);

    thetaCmd = controllerAngle(x,ballVelocity,p);
    beamAngleRate = (thetaCmd-beamAngle)/p.tau;

    % Solid sphere rolling without slipping: acceleration factor = 5/7.
    % Positive theta raises the right end, so the ball accelerates left.
    ballAcceleration = -(5/7)*p.g*sin(beamAngle) ...
                       - p.damping*ballVelocity;

    % Keep the ball within the usable beam travel.
    if x >= p.xLimit && ballVelocity > 0
        ballAcceleration = min(ballAcceleration,0);
    elseif x <= -p.xLimit && ballVelocity < 0
        ballAcceleration = max(ballAcceleration,0);
    end

    dydt = [ballVelocity; ballAcceleration; beamAngleRate];
end

function thetaCmd = controllerAngle(x,ballVelocity,p)
    positionError = x-p.xTarget;
    thetaCmd = p.Kp*positionError + p.Kd*ballVelocity;
    thetaCmd = min(max(thetaCmd,-p.thetaMax),p.thetaMax);
end

function [V,F] = boxMesh(dimensions)
    hx = dimensions(1)/2;
    hy = dimensions(2)/2;
    hz = dimensions(3)/2;

    V = [-hx -hy -hz;
          hx -hy -hz;
          hx  hy -hz;
         -hx  hy -hz;
         -hx -hy  hz;
          hx -hy  hz;
          hx  hy  hz;
         -hx  hy  hz];

    F = [1 2 3 4;
         5 8 7 6;
         1 5 6 2;
         2 6 7 3;
         3 7 8 4;
         4 8 5 1];
end

function Vworld = transformBeam(Vlocal,theta,pivotZ)
    c = cos(theta);
    s = sin(theta);

    % Rotation about Y, chosen so positive theta raises the right end.
    xw = Vlocal(:,1)*c - Vlocal(:,3)*s;
    yw = Vlocal(:,2);
    zw = Vlocal(:,1)*s + Vlocal(:,3)*c + pivotZ;

    Vworld = [xw yw zw];
end

function h = makeBox(ax,dimensions,center,faceColor)
    [V,F] = boxMesh(dimensions);
    V(:,1) = V(:,1)+center(1);
    V(:,2) = V(:,2)+center(2);
    V(:,3) = V(:,3)+center(3);

    h = patch(ax,'Vertices',V,'Faces',F, ...
        'FaceColor',faceColor, ...
        'EdgeColor',[0.08 0.09 0.11], ...
        'LineWidth',0.8);
end
