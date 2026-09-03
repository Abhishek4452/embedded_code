clc;
clear;
close all;

%% =========================================================
%                 WATER CANAL PARAMETERS
% ==========================================================

canalLength = 10;          % Canal length (m)
canalDepth  = 3;           % Maximum water depth (m)

% Initial water level
waterLevel = 0.8;          % m

% Water flow parameters
Qin  = 1.0;                % Inflow (m^3/s)
Qout = 0.4;                % Outflow (m^3/s)

% Effective canal area
A = 3;


%% =========================================================
%                 SIMULATION PARAMETERS
% ==========================================================

dt = 0.02;                 % Sampling time (s)
simulationTime = 30;       % Simulation duration (s)

time = 0:dt:simulationTime;

N = length(time);


%% =========================================================
%                 FLOATING BALL PARAMETERS
% ==========================================================

ballRadius = 0.18;         % Ball radius (m)

ballX = canalLength/2;

% Vertical oscillation
verticalAmplitude = 0.08;  % m
verticalFrequency = 2.0;   % Hz

% Horizontal motion
horizontalAmplitude = 0.5; % m
horizontalFrequency = 0.3; % Hz

% Ball rotation
rotationAmplitude = 8;     % degrees
rotationFrequency = 0.8;   % Hz


%% =========================================================
%                 IMU PARAMETERS
% ==========================================================

g = 9.81;                  % Gravity (m/s^2)

% Accelerometer noise
accNoise = 0.05;           % m/s^2

% Gyroscope noise
gyroNoise = 0.02;          % rad/s


%% =========================================================
%                 DATA STORAGE
% ==========================================================

% Water
waterLevelData = zeros(1,N);

% Ball position
ballXData = zeros(1,N);
ballYData = zeros(1,N);

% Accelerometer
axData = zeros(1,N);
ayData = zeros(1,N);
azData = zeros(1,N);

% Gyroscope
wxData = zeros(1,N);
wyData = zeros(1,N);
wzData = zeros(1,N);

% Ball orientation
rollData  = zeros(1,N);
pitchData = zeros(1,N);

% Total acceleration
totalAcceleration = zeros(1,N);

% Total angular velocity
totalAngularVelocity = zeros(1,N);

% Acceleration RMS
accelerationRMS = zeros(1,N);

% Gravity-removed vibration acceleration
vibrationAcceleration = zeros(1,N);

% Vibration RMS
vibrationRMS = zeros(1,N);


%% =========================================================
%                 CREATE CANAL FIGURE
% ==========================================================

figure(1);

set(gcf,'Color','white');
set(gcf,'Name','Virtual Water Canal');

axis([0 canalLength 0 canalDepth]);
axis manual;

xlabel('Canal Length (m)');
ylabel('Water Depth (m)');

grid on;
hold on;

title('Virtual Water Canal with Floating IMU Ball');


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
%                 SIMULATION LOOP
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

    ballXData(k) = ballX;


    %% -----------------------------------------------------
    % BALL VERTICAL MOTION
    % ------------------------------------------------------

    verticalMotion = ...
        verticalAmplitude * ...
        sin(2*pi*verticalFrequency*t);

    ballY = waterLevel + verticalMotion;

    ballYData(k) = ballY;


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


    % Add accelerometer noise
    ax = ax + accNoise*randn;
    ay = ay + accNoise*randn;
    az = az + accNoise*randn;


    %% -----------------------------------------------------
    % Store accelerometer data
    % ------------------------------------------------------

    axData(k) = ax;
    ayData(k) = ay;
    azData(k) = az;


    %% =====================================================
    %              VIRTUAL GYROSCOPE
    % =====================================================

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


    %% -----------------------------------------------------
    % Store gyroscope data
    % ------------------------------------------------------

    wxData(k) = wx;
    wyData(k) = wy;
    wzData(k) = wz;


    %% =====================================================
    %              TOTAL ACCELERATION
    % =====================================================

    totalAcceleration(k) = ...
        sqrt(ax^2 + ay^2 + az^2);


    %% =====================================================
    %              TOTAL ANGULAR VELOCITY
    % =====================================================

    totalAngularVelocity(k) = ...
        sqrt(wx^2 + wy^2 + wz^2);


    %% =====================================================
    %              ACCELERATION RMS
    % =====================================================

    accelerationRMS(k) = ...
        sqrt(mean(totalAcceleration(1:k).^2));


    %% =====================================================
    %              GRAVITY-REMOVED VIBRATION
    % =====================================================

    vibrationAcceleration(k) = ...
        totalAcceleration(k) - g;


    %% =====================================================
    %              VIBRATION RMS
    % =====================================================

    vibrationRMS(k) = ...
        sqrt(mean(vibrationAcceleration(1:k).^2));


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
         'Water Level = %.2f m | ax = %.2f | ay = %.2f | az = %.2f m/s^2\n' ...
         'Acceleration RMS = %.2f m/s^2 | Vibration RMS = %.2f m/s^2'], ...
         t, ...
         waterLevel, ...
         ax, ay, az, ...
         accelerationRMS(k), ...
         vibrationRMS(k)));


    %% -----------------------------------------------------
    % Update animation
    % ------------------------------------------------------

    drawnow;

end


%% =========================================================
%        ONE PAGE - ALL ANALYSIS GRAPHS
% ==========================================================

figure(2);

set(gcf,'Color','white');
set(gcf,'Name','Complete IMU Analysis');

% Make the figure large enough for all graphs
set(gcf,'Units','normalized');
set(gcf,'Position',[0.02 0.03 0.96 0.92]);


%% =========================================================
%                 1. ACCELEROMETER X
% ==========================================================

subplot(3,3,1);

plot(time,axData,'LineWidth',1);

grid on;

xlabel('Time (s)');
ylabel('m/s^2');

title('Accelerometer X');


%% =========================================================
%                 2. ACCELEROMETER Y
% ==========================================================

subplot(3,3,2);

plot(time,ayData,'LineWidth',1);

grid on;

xlabel('Time (s)');
ylabel('m/s^2');

title('Accelerometer Y');


%% =========================================================
%                 3. ACCELEROMETER Z
% ==========================================================

subplot(3,3,3);

plot(time,azData,'LineWidth',1);

grid on;

xlabel('Time (s)');
ylabel('m/s^2');

title('Accelerometer Z');


%% =========================================================
%                 4. GYROSCOPE X
% ==========================================================

subplot(3,3,4);

plot(time,wxData,'LineWidth',1);

grid on;

xlabel('Time (s)');
ylabel('rad/s');

title('Gyroscope X');


%% =========================================================
%                 5. GYROSCOPE Y
% ==========================================================

subplot(3,3,5);

plot(time,wyData,'LineWidth',1);

grid on;

xlabel('Time (s)');
ylabel('rad/s');

title('Gyroscope Y');


%% =========================================================
%                 6. GYROSCOPE Z
% ==========================================================

subplot(3,3,6);

plot(time,wzData,'LineWidth',1);

grid on;

xlabel('Time (s)');
ylabel('rad/s');

title('Gyroscope Z');


%% =========================================================
%                 7. TOTAL ACCELERATION
% ==========================================================

subplot(3,3,7);

plot(time,totalAcceleration,'LineWidth',1.2);

grid on;

xlabel('Time (s)');
ylabel('m/s^2');

title('Total Acceleration');


%% =========================================================
%                 8. ACCELERATION RMS
% ==========================================================

subplot(3,3,8);

plot(time,accelerationRMS,'LineWidth',1.5);

grid on;

xlabel('Time (s)');
ylabel('m/s^2');

title('Acceleration RMS');


%% =========================================================
%                 9. VIBRATION RMS
% ==========================================================

subplot(3,3,9);

plot(time,vibrationRMS,'LineWidth',1.5);

grid on;

xlabel('Time (s)');
ylabel('m/s^2');

title('Water-Induced Vibration RMS');


%% =========================================================
%                 OVERALL TITLE
% ==========================================================

sgtitle('Floating Ball IMU Analysis','FontSize',16,'FontWeight','bold');


%% =========================================================
%              WATER LEVEL FIGURE
% ==========================================================

figure(3);

set(gcf,'Color','white');
set(gcf,'Name','Water Level');

plot(time,waterLevelData,'LineWidth',2);

grid on;

xlabel('Time (s)');
ylabel('Water Level (m)');

title('Water Level vs Time');


%% =========================================================
%              FINAL RESULTS
% ==========================================================

fprintf('\n');
fprintf('============================================\n');
fprintf('          SIMULATION RESULTS\n');
fprintf('============================================\n');

fprintf('Final Water Level      = %.3f m\n', ...
        waterLevelData(end));

fprintf('Final Acceleration RMS = %.3f m/s^2\n', ...
        accelerationRMS(end));

fprintf('Final Vibration RMS    = %.3f m/s^2\n', ...
        vibrationRMS(end));

fprintf('Maximum Vibration Acc. = %.3f m/s^2\n', ...
        max(abs(vibrationAcceleration)));

fprintf('============================================\n');