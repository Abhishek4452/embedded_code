clc;
clear;
close all;

%% ================= CANAL PARAMETERS =================

canalLength = 10;      % Canal length (m)
canalDepth  = 3;       % Maximum canal depth (m)

% Water
waterLevel = 0.8;      % Initial water level (m)
Qin = 1.0;             % Inflow (m^3/s)
Qout = 0.4;            % Outflow (m^3/s)

% Effective cross-sectional area
A = 3;                 % Canal width (m)

%% ================= SIMULATION PARAMETERS =================

dt = 0.03;
simulationTime = 30;

% Floating ball
ballRadius = 0.18;
ballX = canalLength/2;

% Ball oscillation
shakeAmplitude = 0.08;     % Vertical shaking amplitude (m)
shakeFrequency = 2.5;      % Shaking frequency (Hz)

%% ================= CREATE FIGURE =================

figure('Color','white');

axis([0 canalLength 0 canalDepth]);
axis manual;

xlabel('Canal Length (m)');
ylabel('Water Depth (m)');

grid on;
hold on;

title('Virtual Water Canal');

%% ================= CANAL =================

% Canal bottom
plot([0 canalLength], [0 0], ...
    'k', 'LineWidth', 4);

% Left wall
plot([0 0], [0 canalDepth], ...
    'k', 'LineWidth', 4);

% Right wall
plot([canalLength canalLength], ...
     [0 canalDepth], ...
     'k', 'LineWidth', 4);

%% ================= WATER =================

water = patch( ...
    [0 canalLength canalLength 0], ...
    [0 0 waterLevel waterLevel], ...
    'b');

water.FaceAlpha = 0.5;
water.EdgeColor = 'none';

%% ================= WATER FLOW ARROWS =================

quiver(-1, waterLevel, 1, 0, ...
       'LineWidth', 2, ...
       'MaxHeadSize', 0.5);

text(-1, waterLevel + 0.25, ...
     'Water Inlet', ...
     'FontSize', 11);

quiver(canalLength, waterLevel, 1, 0, ...
       'LineWidth', 2, ...
       'MaxHeadSize', 0.5);

text(canalLength-1, waterLevel + 0.25, ...
     'Water Outlet', ...
     'FontSize', 11);

%% ================= FLOATING BALL =================

theta = linspace(0,2*pi,50);

% Start ball exactly on water surface
ballY = waterLevel;

ball = patch( ...
    ballX + ballRadius*cos(theta), ...
    ballY + ballRadius*sin(theta), ...
    'r');

ball.EdgeColor = 'k';

%% ================= WATER LEVEL LINE =================

levelLine = plot( ...
    [0 canalLength], ...
    [waterLevel waterLevel], ...
    '--k', ...
    'LineWidth', 1.2);

%% ================= SIMULATION =================

for t = 0:dt:simulationTime

    %% Water-level equation
    %
    % A dh/dt = Qin - Qout
    %

    dh = ((Qin - Qout)/A) * dt;

    waterLevel = waterLevel + dh;

    %% Keep water inside canal

    if waterLevel > canalDepth
        waterLevel = canalDepth;
    end

    if waterLevel < 0
        waterLevel = 0;
    end

    %% ================= UPDATE WATER =================

    set(water, ...
        'YData', ...
        [0 0 waterLevel waterLevel]);

    %% ================= FLOATING BALL =================

    % Small shaking caused by water movement
    shake = shakeAmplitude * ...
            sin(2*pi*shakeFrequency*t);

    % Ball follows water level
    ballY = waterLevel + shake;

    % Prevent ball from going below water surface
    if ballY < waterLevel
        ballY = waterLevel;
    end

    % Update ball
    set(ball, ...
        'XData', ...
        ballX + ballRadius*cos(theta), ...
        'YData', ...
        ballY + ballRadius*sin(theta));

    %% ================= WATER LEVEL INDICATOR =================

    set(levelLine, ...
        'YData', ...
        [waterLevel waterLevel]);

    %% ================= TITLE =================

    title(sprintf( ...
        'Virtual Water Canal   |   Time = %.1f s   |   Water Level = %.2f m', ...
        t, waterLevel));

    %% Update screen

    drawnow;

    pause(0.01);

end