clc;
clear;
close all;

%% =========================================================
%                 WATER CANAL PARAMETERS
% ==========================================================

canalLength = 10;        % Canal length (m)
canalDepth  = 3;         % Maximum water depth (m)

% Water parameters
waterLevel = 0.8;        % Initial water level (m)

Qin  = 1.0;              % Inflow (m^3/s)
Qout = 0.4;              % Outflow (m^3/s)

A = 3;                   % Effective canal area

%% =========================================================
%                 SIMULATION PARAMETERS
% ==========================================================

dt = 0.02;               % Sampling time
simulationTime = 30;     % Simulation duration

time = 0:dt:simulationTime;

%% =========================================================
%                 FLOATING BALL
% ==========================================================

ballRadius = 0.18;

ballX = canalLength/2;

% Vertical oscillation
verticalAmplitude = 0.08;
verticalFrequency = 2.0;

% Horizontal motion
horizontalAmplitude = 0.5;
horizontalFrequency = 0.3;

% Ball rotation
rotationAmplitude = 8;       % degrees
rotationFrequency = 0.8;

%% =========================================================
%                 IMU PARAMETERS
% ==========================================================

g = 9.81;                    % Gravity

% Accelerometer noise
accNoise = 0.05;

% Gyroscope noise
gyroNoise = 0.02;

%% =========================================================
%                 DATA STORAGE
% ==========================================================

N = length(time);

waterLevelData = zeros(1,N);

ballXData = zeros(1,N);
ballYData = zeros(1,N);

axData = zeros(1,N);
ayData = zeros(1,N);
azData = zeros(1,N);

wxData = zeros(1,N);
wyData = zeros(1,N);
wzData = zeros(1,N);

rollData  = zeros(1,N);
pitchData = zeros(1,N);

totalAcceleration = zeros(1,N);
totalAngularVelocity = zeros(1,N);

%% =========================================================
%                 CREATE FIGURE
% ==========================================================

figure('Color','white');

axis([0 canalLength 0 canalDepth]);

axis manual;

xlabel('Canal Length (m)');
ylabel('Water Depth (m)');

grid on;
hold on;

title('Virtual Water Canal with IMU Floating Ball');

%% =========================================================
%                 CANAL
% ==========================================================

% Bottom
plot([0 canalLength], ...
     [0 0], ...
     'k', ...
     'LineWidth',4);

% Left wall
plot([0 0], ...
     [0 canalDepth], ...
     'k', ...
     'LineWidth',4);

% Right wall
plot([canalLength canalLength], ...
     [0 canalDepth], ...
     'k', ...
     'LineWidth',4);

%% =========================================================
%                 WATER
% ==========================================================

water = patch( ...
    [0 canalLength canalLength 0], ...
    [0 0 waterLevel waterLevel], ...
    'b');

water.FaceAlpha = 0.5;
water.EdgeColor = 'none';

%% =========================================================
%                 WATER FLOW
% ==========================================================

quiver(-1, waterLevel, ...
       1, 0, ...
       'LineWidth',2, ...
       'MaxHeadSize',0.5);

text(-1, waterLevel+0.25, ...
     'Water Inlet', ...
     'FontSize',11);

quiver(canalLength, waterLevel, ...
       1, 0, ...
       'LineWidth',2, ...
       'MaxHeadSize',0.5);

text(canalLength-1, waterLevel+0.25, ...
     'Water Outlet', ...
     'FontSize',11);

%% =========================================================
%                 BALL
% ==========================================================

theta = linspace(0,2*pi,50);

ballY = waterLevel;

ball = patch( ...
    ballX + ballRadius*cos(theta), ...
    ballY + ballRadius*sin(theta), ...
    'r');

ball.EdgeColor = 'k';

%% =========================================================
%                 WATER LEVEL LINE
% ==========================================================

levelLine = plot( ...
    [0 canalLength], ...
    [waterLevel waterLevel], ...
    '--k', ...
    'LineWidth',1.2);

%% =========================================================
%                 SIMULATION
% ==========================================================

for k = 1:N

    t = time(k);

    %% -----------------------------------------------------
    % WATER LEVEL
    % ------------------------------------------------------

    dh = ((Qin - Qout)/A) * dt;

    waterLevel = waterLevel + dh;

    % Keep water inside canal
    if waterLevel > canalDepth
        waterLevel = canalDepth;
    end

    if waterLevel < 0
        waterLevel = 0;
    end

    waterLevelData(k) = waterLevel;

    %% -----------------------------------------------------
    % BALL HORIZONTAL MOTION
    % ------------------------------------------------------

    ballX = canalLength/2 + ...
        horizontalAmplitude * ...
        sin(2*pi*horizontalFrequency*t);

    %% -----------------------------------------------------
    % BALL VERTICAL MOTION
    % ------------------------------------------------------

    verticalMotion = ...
        verticalAmplitude * ...
        sin(2*pi*verticalFrequency*t);

    ballY = waterLevel + verticalMotion;

    %% -----------------------------------------------------
    % BALL ROTATION
    % ------------------------------------------------------

    roll = rotationAmplitude * ...
        sin(2*pi*rotationFrequency*t);

    pitch = rotationAmplitude * ...
        cos(2*pi*rotationFrequency*t);

    rollData(k) = roll;
    pitchData(k) = pitch;

    %% =====================================================
    %              VIRTUAL ACCELEROMETER
    % =====================================================

    % Vertical acceleration
    az = g + ...
        -(verticalAmplitude * ...
        (2*pi*verticalFrequency)^2) * ...
        sin(2*pi*verticalFrequency*t);

    % Horizontal acceleration
    ax = ...
        -(horizontalAmplitude * ...
        (2*pi*horizontalFrequency)^2) * ...
        sin(2*pi*horizontalFrequency*t);

    % Sideways acceleration
    ay = ...
        0.1 * sin(2*pi*1.5*t);

    % Add sensor noise
    ax = ax + accNoise*randn;
    ay = ay + accNoise*randn;
    az = az + accNoise*randn;

    %% Store accelerometer data

    axData(k) = ax;
    ayData(k) = ay;
    azData(k) = az;

    %% =====================================================
    %              VIRTUAL GYROSCOPE
    % =====================================================

    % Angular velocity

    wx = rotationAmplitude * ...
        pi/180 * ...
        (2*pi*rotationFrequency) * ...
        cos(2*pi*rotationFrequency*t);

    wy = -rotationAmplitude * ...
        pi/180 * ...
        (2*pi*rotationFrequency) * ...
        sin(2*pi*rotationFrequency*t);

    wz = 0.2 * sin(2*pi*0.5*t);

    % Add gyro noise
    wx = wx + gyroNoise*randn;
    wy = wy + gyroNoise*randn;
    wz = wz + gyroNoise*randn;

    %% Store gyroscope data

    wxData(k) = wx;
    wyData(k) = wy;
    wzData(k) = wz;

    %% =====================================================
    %              IMU MAGNITUDES
    % =====================================================

    totalAcceleration(k) = ...
        sqrt(ax^2 + ay^2 + az^2);

    totalAngularVelocity(k) = ...
        sqrt(wx^2 + wy^2 + wz^2);

    %% =====================================================
    %              UPDATE WATER
    % =====================================================

    set(water, ...
        'YData', ...
        [0 0 waterLevel waterLevel]);

    %% =====================================================
    %              UPDATE BALL
    % =====================================================

    set(ball, ...
        'XData', ...
        ballX + ballRadius*cos(theta), ...
        'YData', ...
        ballY + ballRadius*sin(theta));

    %% =====================================================
    %              UPDATE WATER LEVEL
    % =====================================================

    set(levelLine, ...
        'YData', ...
        [waterLevel waterLevel]);

    %% =====================================================
    %              DISPLAY IMU VALUES
    % =====================================================

    title(sprintf( ...
        ['Water Canal + Virtual IMU | Time = %.1f s\n' ...
         'Level = %.2f m | ax = %.2f | ay = %.2f | az = %.2f m/s^2'], ...
         t, waterLevel, ax, ay, az));

    %% Update screen

    drawnow;

end

%% =========================================================
%              IMU DATA PLOTS
% ==========================================================

figure('Color','white');

%% ---------------------------------------------------------
% ACCELEROMETER
% ----------------------------------------------------------

subplot(3,1,1);

plot(time, axData, 'LineWidth',1.2);

grid on;

xlabel('Time (s)');
ylabel('a_x (m/s^2)');

title('Accelerometer X');

%% ---------------------------------------------------------

subplot(3,1,2);

plot(time, ayData, 'LineWidth',1.2);

grid on;

xlabel('Time (s)');
ylabel('a_y (m/s^2)');

title('Accelerometer Y');

%% ---------------------------------------------------------

subplot(3,1,3);

plot(time, azData, 'LineWidth',1.2);

grid on;

xlabel('Time (s)');
ylabel('a_z (m/s^2)');

title('Accelerometer Z');

%% =========================================================
%              GYROSCOPE
% ==========================================================

figure('Color','white');

subplot(3,1,1);

plot(time, wxData, 'LineWidth',1.2);

grid on;

xlabel('Time (s)');
ylabel('\omega_x (rad/s)');

title('Gyroscope X');

subplot(3,1,2);

plot(time, wyData, 'LineWidth',1.2);

grid on;

xlabel('Time (s)');
ylabel('\omega_y (rad/s)');

title('Gyroscope Y');

subplot(3,1,3);

plot(time, wzData, 'LineWidth',1.2);

grid on;

xlabel('Time (s)');
ylabel('\omega_z (rad/s)');

title('Gyroscope Z');

%% =========================================================
%              TOTAL IMU MOTION
% ==========================================================

figure('Color','white');

subplot(2,1,1);

plot(time, totalAcceleration, ...
     'LineWidth',1.5);

grid on;

xlabel('Time (s)');
ylabel('Acceleration (m/s^2)');

title('Total Acceleration');

subplot(2,1,2);

plot(time, totalAngularVelocity, ...
     'LineWidth',1.5);

grid on;

xlabel('Time (s)');
ylabel('Angular Velocity (rad/s)');

title('Total Angular Velocity');

%% =========================================================
%              WATER LEVEL
% ==========================================================

figure('Color','white');

plot(time, waterLevelData, ...
     'LineWidth',2);

grid on;

xlabel('Time (s)');
ylabel('Water Level (m)');

title('Water Level vs Time');