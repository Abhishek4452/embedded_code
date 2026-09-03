%% ================================================================
%  VERTICALLY CONSTRAINED FLOATING BALL IN A LOW-FLOW WATER CANAL
%
%  Single-degree-of-freedom vertical fluid-structure interaction model
%
%  Governing equation:
%
%       meff*x_ddot + c*x_dot + k*x = Fw(t)
%
%  where:
%       x       = vertical displacement from equilibrium [m]
%       x_dot   = vertical velocity [m/s]
%       x_ddot  = vertical acceleration [m/s^2]
%       meff    = effective moving mass [kg]
%       c       = damping coefficient [N.s/m]
%       k       = equivalent vertical restoring stiffness [N/m]
%
%  Water excitation:
%
%       Fw(t) = Aw*sin(2*pi*fw*t)
%             + Av*sin(2*pi*fv*t)
%             + Fn(t)
%
%  Components:
%       1. Low-frequency water-surface disturbance
%       2. Vortex-shedding excitation
%       3. Filtered stochastic turbulence
%
%  IMPORTANT PHYSICAL INTERPRETATION:
%
%  - Mean water flow by itself does NOT continuously vibrate the ball.
%  - Mean velocity is used to estimate the dynamic excitation.
%  - Buoyancy, gravity and static forces are absorbed into the
%    equilibrium position and equivalent restoring stiffness.
%  - Horizontal motion is completely constrained.
%  - Only vertical motion is allowed.
%  - The turbulence model is an engineering approximation, not CFD.
%  - Vortex shedding is represented by a sinusoidal force using
%    the Strouhal relation.
%  - The water-surface motion is also represented phenomenologically.
%
%  ================================================================

clc;
clear;
close all;

%% ================================================================
% 1. USER-ADJUSTABLE PHYSICAL PARAMETERS
% ================================================================

%% Ball parameters
m_ball = 0.030;          % Physical ball mass [kg]
m_added = 0.005;         % Approx. added hydrodynamic mass [kg]

% Effective mass
meff = m_ball + m_added; % [kg]

ball_diameter = 0.060;   % Ball diameter [m]
ball_radius   = ball_diameter/2;

%% Equivalent vertical constraint
%
% This spring-damper represents:
%   - tether/support flexibility
%   - effective buoyancy restoring force
%   - structural restoring effects
%
k = 12.0;                % Equivalent stiffness [N/m]
c = 0.080;               % Equivalent damping [N.s/m]

%% Water-flow parameters
U = 0.25;                % Mean water velocity [m/s]
rho = 1000;              % Water density [kg/m^3]

%% Water surface disturbance
Aw = 0.030;              % Water-surface excitation force amplitude [N]
fw = 0.15;               % Water-surface disturbance frequency [Hz]

water_surface_amp = 0.004;  % Physical water-level variation [m]

%% Vortex shedding
St = 0.20;               % Strouhal number for bluff spherical object

% Vortex shedding frequency:
fv = St * U / ball_diameter;

% Vortex excitation amplitude
Av = 0.020;              % [N]

%% Turbulence
Fn_rms = 0.008;          % Approx. RMS turbulence force [N]

% Turbulence correlation / cutoff frequency
f_turb = 2.0;            % [Hz]

%% Initial transient condition
initial_displacement = 0.020;  % Initial vertical displacement [m]
initial_velocity     = 0.000;  % Initial vertical velocity [m/s]

%% Simulation times
T_transient = 8;         % Initial free-decay period [s]
T_sustained = 32;        % Forced-vibration period [s]

T_total = T_transient + T_sustained;

%% Numerical sampling
Fs = 200;                % Sampling frequency [Hz]
dt = 1/Fs;

t1 = (0:dt:T_transient).';
t2 = (T_transient+dt:dt:T_total).';

%% Moving RMS window
RMS_window_seconds = 2.0;
RMS_window_samples = round(RMS_window_seconds * Fs);

%% Random turbulence seed
rng(10);

%% ================================================================
% 2. CALCULATE IMPORTANT SYSTEM PROPERTIES
% ================================================================

% Undamped natural frequency
fn_natural = (1/(2*pi))*sqrt(k/meff);

% Natural angular frequency
wn_natural = sqrt(k/meff);

% Damping ratio
zeta = c/(2*sqrt(k*meff));

% Damped natural frequency
if zeta < 1
    fd_natural = fn_natural*sqrt(1-zeta^2);
else
    fd_natural = 0;
end

%% ================================================================
% 3. GENERATE FILTERED STOCHASTIC TURBULENCE
% ================================================================
%
% The random force is NOT white noise directly.
%
% A first-order low-pass filter is used to represent the fact that
% real turbulent hydrodynamic fluctuations have finite correlation
% time and are therefore not infinitely fast.
%
% This is an engineering approximation.

N2 = length(t2);

raw_noise = randn(N2,1);

% First-order low-pass filter coefficient
tau_turb = 1/(2*pi*f_turb);
alpha = dt/(tau_turb + dt);

filtered_noise = zeros(N2,1);

for i = 2:N2
    filtered_noise(i) = ...
        filtered_noise(i-1) + ...
        alpha*(raw_noise(i)-filtered_noise(i-1));
end

% Normalize to requested RMS
current_rms = sqrt(mean(filtered_noise.^2));

if current_rms > 0
    filtered_noise = ...
        filtered_noise/current_rms * Fn_rms;
end

Fn = filtered_noise;

%% ================================================================
% 4. SUSTAINED HYDRODYNAMIC FORCE
% ================================================================

% Time relative to beginning of sustained phase
t_forcing = t2 - T_transient;

% Water-surface component
Fw_surface = Aw*sin(2*pi*fw*t_forcing);

% Vortex-shedding component
Fw_vortex = Av*sin(2*pi*fv*t_forcing);

% Filtered turbulent component
Fw_turb = Fn;

% Total fluctuating water force
Fw2 = Fw_surface + Fw_vortex + Fw_turb;

%% ================================================================
% 5. INITIAL TRANSIENT PHASE
% ================================================================
%
% During this phase:
%
%       Fw(t) = 0
%
% Therefore the ball undergoes natural free vibration.
%
% The initial displacement causes the ball to oscillate and damping
% causes the oscillation amplitude to decay naturally.

ode_options = odeset('RelTol',1e-7,'AbsTol',1e-9);

x0 = [initial_displacement; initial_velocity];

[t1_sol, y1] = ode45( ...
    @(t,y) vertical_ball_ode(t,y,meff,c,k, ...
    @(tt) 0), ...
    t1, x0, ode_options);

x1 = y1(:,1);
v1 = y1(:,2);

% Calculate acceleration directly from governing equation
a1 = zeros(size(x1));

for i = 1:length(t1_sol)
    Fcurrent = 0;
    a1(i) = (Fcurrent - c*v1(i) - k*x1(i))/meff;
end

%% ================================================================
% 6. SUSTAINED EXCITATION PHASE
% ================================================================

% Interpolation function for turbulent excitation
force_function = @(tt) ...
    interp1(t2,Fw2,tt,'linear','extrap');

% Start sustained phase from final transient condition
x0_forced = [x1(end); v1(end)];

[t2_sol, y2] = ode45( ...
    @(t,y) vertical_ball_ode( ...
    t,y,meff,c,k,force_function), ...
    t2, x0_forced, ode_options);

x2 = y2(:,1);
v2 = y2(:,2);

% Calculate acceleration from equation of motion
a2 = zeros(size(x2));

for i = 1:length(t2_sol)

    Fcurrent = force_function(t2_sol(i));

    a2(i) = ...
        (Fcurrent - c*v2(i) - k*x2(i))/meff;

end

%% ================================================================
% 7. COMBINE BOTH PHASES
% ================================================================

% Remove duplicate transient endpoint
t = [t1_sol; t2_sol];

x = [x1; x2];
v = [v1; v2];
a = [a1; a2];

% Total force for plotting
Fw_total = [zeros(size(t1_sol)); Fw2];

%% ================================================================
% 8. ACCELERATION RMS
% ================================================================

% Overall acceleration RMS
%
%       a_RMS = sqrt( (1/N) * sum(ai^2) )

accel_RMS = sqrt(mean(a.^2));

% Sustained-phase acceleration RMS
accel_RMS_sustained = sqrt(mean(a2.^2));

%% ================================================================
% 9. RUNNING / MOVING ACCELERATION RMS
% ================================================================

running_RMS = movrms(a,RMS_window_samples);

% movrms is available in recent MATLAB versions.
%
% If an older MATLAB version does not have movrms, replace the above
% line with:
%
% running_RMS = sqrt(movmean(a.^2,RMS_window_samples));

%% ================================================================
% 10. PEAK AND PEAK-TO-PEAK ACCELERATION
% ================================================================

peak_acceleration = max(abs(a));

peak_to_peak_acceleration = max(a) - min(a);

%% ================================================================
% 11. FFT OF ACCELERATION
% ================================================================
%
% For vibration identification, the sustained excitation phase is
% more useful than the initial transient.
%
% Therefore the FFT is calculated primarily from a2.

Nfft = length(a2);

% Remove DC component
a2_fft_signal = a2 - mean(a2);

Y = fft(a2_fft_signal);

P2 = abs(Y/Nfft);

P1 = P2(1:floor(Nfft/2)+1);

if length(P1) > 2
    P1(2:end-1) = 2*P1(2:end-1);
end

f_fft = Fs*(0:floor(Nfft/2))/Nfft;

%% ================================================================
% 12. DOMINANT VIBRATION FREQUENCY
% ================================================================

% Ignore DC
if length(P1) > 1
    [~,idx_dom] = max(P1(2:end));
    idx_dom = idx_dom + 1;
else
    idx_dom = 1;
end

dominant_frequency = f_fft(idx_dom);

%% ================================================================
% 13. POWER SPECTRAL DENSITY
% ================================================================
%
% We calculate a simple one-sided periodogram manually so that the
% script does not require the Signal Processing Toolbox.

PSD_two_sided = abs(Y).^2/(Fs*Nfft);

PSD_one_sided = PSD_two_sided(1:floor(Nfft/2)+1);

if length(PSD_one_sided) > 2
    PSD_one_sided(2:end-1) = ...
        2*PSD_one_sided(2:end-1);
end

f_psd = f_fft;

%% ================================================================
% 14. WATER LEVEL DISTURBANCE
% ================================================================

water_level = zeros(size(t));

% During transient, water surface is nearly undisturbed
water_level(1:length(t1_sol)) = 0;

% During forced phase
water_level(length(t1_sol)+1:end) = ...
    water_surface_amp * ...
    sin(2*pi*fw*t_forcing);

%% ================================================================
% 15. COMMAND-WINDOW SUMMARY
% ================================================================

fprintf('\n');
fprintf('============================================================\n');
fprintf(' VERTICAL FLOATING BALL - SIMULATION SUMMARY\n');
fprintf('============================================================\n');

fprintf('Ball mass                    : %.4f kg\n',m_ball);
fprintf('Added hydrodynamic mass      : %.4f kg\n',m_added);
fprintf('Effective mass               : %.4f kg\n',meff);
fprintf('Equivalent stiffness         : %.4f N/m\n',k);
fprintf('Damping coefficient          : %.4f N.s/m\n',c);

fprintf('\n');
fprintf('Mean water velocity          : %.4f m/s\n',U);
fprintf('Ball diameter                : %.4f m\n',ball_diameter);

fprintf('\n');
fprintf('Natural frequency             : %.4f Hz\n',fn_natural);
fprintf('Damped natural frequency     : %.4f Hz\n',fd_natural);
fprintf('Damping ratio                : %.4f\n',zeta);

fprintf('\n');
fprintf('Water disturbance frequency  : %.4f Hz\n',fw);
fprintf('Vortex shedding frequency    : %.4f Hz\n',fv);
fprintf('Dominant vibration frequency : %.4f Hz\n',dominant_frequency);

fprintf('\n');
fprintf('Overall acceleration RMS     : %.6f m/s^2\n',accel_RMS);
fprintf('Sustained acceleration RMS   : %.6f m/s^2\n',accel_RMS_sustained);
fprintf('Peak acceleration            : %.6f m/s^2\n',peak_acceleration);
fprintf('Peak-to-peak acceleration    : %.6f m/s^2\n', ...
    peak_to_peak_acceleration);

fprintf('============================================================\n');
fprintf('\n');

%% ================================================================
% 16. CREATE MAIN FIGURE
% ================================================================
%
% Everything is placed into one tiled figure.
%
% Tile 1  : Virtual canal + ball
% Tile 2  : Water disturbance
% Tile 3  : Hydrodynamic force
% Tile 4  : Displacement
% Tile 5  : Velocity
% Tile 6  : Acceleration
% Tile 7  : Running acceleration RMS
% Tile 8  : FFT / frequency spectrum

fig = figure( ...
    'Name','Vertical Floating Ball - Low Flow Canal', ...
    'Color','w', ...
    'Position',[30 30 1500 850]);

tl = tiledlayout(fig,4,2, ...
    'TileSpacing','compact', ...
    'Padding','compact');

sgtitle(tl, ...
    'Vertically Constrained Floating Ball - Low-Flow Water Canal', ...
    'FontSize',16, ...
    'FontWeight','bold');

%% ================================================================
% TILE 1 - VIRTUAL CANAL
% ================================================================

ax1 = nexttile(tl,1);

hold(ax1,'on');
grid(ax1,'on');

% Canal dimensions
canal_width = 1.20;
canal_bottom = 0;
canal_top = 0.85;

% Nominal water surface
water_nominal = 0.48;

% Canal boundary
plot(ax1, ...
    [-canal_width/2 canal_width/2], ...
    [canal_bottom canal_bottom], ...
    'k','LineWidth',3);

plot(ax1, ...
    [-canal_width/2 -canal_width/2], ...
    [canal_bottom canal_top], ...
    'k','LineWidth',3);

plot(ax1, ...
    [canal_width/2 canal_width/2], ...
    [canal_bottom canal_top], ...
    'k','LineWidth',3);

% Water patch
water_x = [-canal_width/2 canal_width/2 ...
           canal_width/2 -canal_width/2];

water_y = [water_nominal water_nominal ...
           canal_bottom canal_bottom];

water_patch = patch(ax1, ...
    water_x,water_y, ...
    [0.75 0.88 1.00], ...
    'EdgeColor','none');

% Water surface line
water_line = plot(ax1, ...
    [-canal_width/2 canal_width/2], ...
    [water_nominal water_nominal], ...
    'b','LineWidth',2);

% Fixed overhead support
support_y = 0.78;

plot(ax1, ...
    [-0.18 0.18], ...
    [support_y support_y], ...
    'k','LineWidth',6);

% Vertical guide / constraint
plot(ax1, ...
    [0 0], ...
    [support_y-ball_radius support_y-0.08], ...
    'k--','LineWidth',1);

% Initial ball equilibrium location
ball_eq_y = water_nominal + 0.010;

% Ball
ball_plot = plot(ax1, ...
    0,ball_eq_y, ...
    'o', ...
    'MarkerSize',20, ...
    'MarkerFaceColor',[1 0.5 0.1], ...
    'MarkerEdgeColor','k', ...
    'LineWidth',1.5);

% Spring/damper visual
spring_line = plot(ax1, ...
    [0 0], ...
    [support_y ball_eq_y], ...
    'k-','LineWidth',1.5);

% Horizontal constraint indication
plot(ax1, ...
    [-0.10 0.10], ...
    [ball_eq_y ball_eq_y], ...
    'k:','LineWidth',1);

xlabel(ax1,'Horizontal position [m]');
ylabel(ax1,'Vertical position [m]');

title(ax1,'Virtual Canal - Ball Constrained to Vertical Motion');

xlim(ax1,[-canal_width/2 canal_width/2]);
ylim(ax1,[canal_bottom canal_top]);

axis(ax1,'manual');

text(ax1, ...
    -0.55,0.72, ...
    sprintf('U = %.2f m/s',U), ...
    'FontSize',9);

text(ax1, ...
    -0.55,0.67, ...
    'Horizontal motion = 0', ...
    'FontSize',9);

%% ================================================================
% TILE 2 - WATER LEVEL
% ================================================================

ax2 = nexttile(tl,2);

plot(ax2,t,water_level*1000, ...
    'b','LineWidth',1.3);

hold(ax2,'on');
xline(ax2,T_transient, ...
    'k--','Transient / forced phase');

xlabel(ax2,'Time [s]');
ylabel(ax2,'Water level disturbance [mm]');

title(ax2,'Water-Surface Disturbance');

grid(ax2,'on');
xlim(ax2,[0 T_total]);

%% ================================================================
% TILE 3 - HYDRODYNAMIC FORCE
% ================================================================

ax3 = nexttile(tl,3);

plot(ax3,t,Fw_total, ...
    'LineWidth',1.2);

hold(ax3,'on');

plot(ax3,t(length(t1_sol)+1:end), ...
    Fw_surface, ...
    '--','LineWidth',0.9);

plot(ax3,t(length(t1_sol)+1:end), ...
    Fw_vortex, ...
    '--','LineWidth',0.9);

plot(ax3,t(length(t1_sol)+1:end), ...
    Fw_turb, ...
    ':','LineWidth',0.8);

xline(ax3,T_transient, ...
    'k--','Transient / forced');

xlabel(ax3,'Time [s]');
ylabel(ax3,'Force [N]');

title(ax3,'Fluctuating Hydrodynamic Excitation');

legend(ax3, ...
    'Total force', ...
    'Water surface', ...
    'Vortex shedding', ...
    'Turbulence', ...
    'Location','best');

grid(ax3,'on');
xlim(ax3,[0 T_total]);

%% ================================================================
% TILE 4 - DISPLACEMENT
% ================================================================

ax4 = nexttile(tl,4);

plot(ax4,t,x*1000, ...
    'LineWidth',1.3);

hold(ax4,'on');

xline(ax4,T_transient, ...
    'k--','Transient / forced');

xlabel(ax4,'Time [s]');
ylabel(ax4,'Vertical displacement [mm]');

title(ax4,'Vertical Displacement');

grid(ax4,'on');
xlim(ax4,[0 T_total]);

%% ================================================================
% TILE 5 - VELOCITY
% ================================================================

ax5 = nexttile(tl,5);

plot(ax5,t,v, ...
    'LineWidth',1.2);

hold(ax5,'on');

xline(ax5,T_transient, ...
    'k--','Transient / forced');

xlabel(ax5,'Time [s]');
ylabel(ax5,'Vertical velocity [m/s]');

title(ax5,'Vertical Velocity');

grid(ax5,'on');
xlim(ax5,[0 T_total]);

%% ================================================================
% TILE 6 - ACCELERATION
% ================================================================

ax6 = nexttile(tl,6);

plot(ax6,t,a, ...
    'LineWidth',1.1);

hold(ax6,'on');

xline(ax6,T_transient, ...
    'k--','Transient / forced');

% RMS annotation
text(ax6, ...
    0.02*T_total, ...
    0.88*max(a), ...
    sprintf('RMS = %.4f m/s^2',accel_RMS), ...
    'FontWeight','bold');

text(ax6, ...
    0.02*T_total, ...
    0.72*max(a), ...
    sprintf('Peak = %.4f m/s^2',peak_acceleration), ...
    'FontWeight','bold');

xlabel(ax6,'Time [s]');
ylabel(ax6,'Acceleration [m/s^2]');

title(ax6,'Vertical Acceleration');

grid(ax6,'on');
xlim(ax6,[0 T_total]);

%% ================================================================
% TILE 7 - RUNNING RMS
% ================================================================

ax7 = nexttile(tl,7);

plot(ax7,t,running_RMS, ...
    'LineWidth',1.4);

hold(ax7,'on');

xline(ax7,T_transient, ...
    'k--','Transient / forced');

yline(ax7,accel_RMS_sustained, ...
    'r--', ...
    sprintf('Sustained RMS = %.4f', ...
    accel_RMS_sustained));

xlabel(ax7,'Time [s]');
ylabel(ax7,'Acceleration RMS [m/s^2]');

title(ax7, ...
    sprintf('Running Acceleration RMS - %.1f s Window', ...
    RMS_window_seconds));

grid(ax7,'on');
xlim(ax7,[0 T_total]);

%% ================================================================
% TILE 8 - FFT / FREQUENCY SPECTRUM
% ================================================================

ax8 = nexttile(tl,8);

plot(ax8,f_fft,P1, ...
    'LineWidth',1.2);

hold(ax8,'on');

% Natural frequency
xline(ax8,fn_natural, ...
    'k--', ...
    sprintf('f_n = %.2f Hz',fn_natural));

% Vortex shedding frequency
xline(ax8,fv, ...
    'r--', ...
    sprintf('f_v = %.2f Hz',fv));

% Water frequency
xline(ax8,fw, ...
    'b--', ...
    sprintf('f_w = %.2f Hz',fw));

% Dominant frequency
xline(ax8,dominant_frequency, ...
    'm-', ...
    sprintf('Dominant = %.2f Hz',dominant_frequency), ...
    'LineWidth',1.5);

xlabel(ax8,'Frequency [Hz]');
ylabel(ax8,'Acceleration amplitude [m/s^2]');

title(ax8,'Acceleration FFT / Frequency Spectrum');

grid(ax8,'on');

xlim(ax8,[0 min(10,Fs/2)]);

%% ================================================================
% 17. ANNOTATION BOX
% ================================================================

annotation(fig,'textbox', ...
    [0.70 0.935 0.27 0.045], ...
    'String',sprintf( ...
    'RMS = %.4f m/s^2   |   Peak = %.4f m/s^2   |   f_n = %.2f Hz   |   f_dom = %.2f Hz', ...
    accel_RMS, ...
    peak_acceleration, ...
    fn_natural, ...
    dominant_frequency), ...
    'FitBoxToText','off', ...
    'HorizontalAlignment','center', ...
    'FontWeight','bold', ...
    'EdgeColor','k');

%% ================================================================
% 18. ANIMATION
% ================================================================
%
% The ball's horizontal coordinate is always:
%
%       x_horizontal = 0
%
% Therefore the animation explicitly demonstrates the vertical
% constraint.
%
% The ball follows:
%
%       y_ball = equilibrium position + x(t)
%
% The water surface follows the small simulated disturbance.
%
% The animation is intentionally slowed slightly so the motion can
% be visually inspected.

fprintf('Starting animation...\n');

animation_skip = max(1,round(Fs/25));

for i = 1:animation_skip:length(t)

    %% Current ball position
    ball_y = ball_eq_y + x(i);

    %% Current water level
    current_water = water_nominal + water_level(i);

    %% Update water patch
    water_patch.XData = ...
        [-canal_width/2 canal_width/2 ...
         canal_width/2 -canal_width/2];

    water_patch.YData = ...
        [current_water current_water ...
         canal_bottom canal_bottom];

    %% Update water surface
    water_line.YData = ...
        [current_water current_water];

    %% Update ball
    ball_plot.XData = 0;
    ball_plot.YData = ball_y;

    %% Update spring / tether
    spring_line.XData = [0 0];
    spring_line.YData = ...
        [support_y ball_y];

    %% Display current time
    title(ax1, ...
        sprintf('Virtual Canal - t = %.2f s',t(i)));

    drawnow limitrate;

end

fprintf('Animation complete.\n');

%% ================================================================
% 19. OPTIONAL STATIC FINAL FRAME
% ================================================================

% Keep final position visible
final_ball_y = ball_eq_y + x(end);
final_water = water_nominal + water_level(end);

ball_plot.XData = 0;
ball_plot.YData = final_ball_y;

water_line.YData = [final_water final_water];

water_patch.YData = ...
    [final_water final_water canal_bottom canal_bottom];

spring_line.YData = [support_y final_ball_y];

drawnow;

%% ================================================================
% 20. LOCAL ODE FUNCTION
% ================================================================
%
% MATLAB allows local functions at the end of a script.
%
% State vector:
%
%       y(1) = x
%       y(2) = x_dot
%
% Therefore:
%
%       x_dot     = y(2)
%
%       x_ddot = (Fw - c*x_dot - k*x)/meff

function dydt = vertical_ball_ode(~,y,meff,c,k,force_function)

    x = y(1);
    v = y(2);

    Fw = force_function(0);

    % Governing equation:
    %
    % meff*x_ddot + c*x_dot + k*x = Fw

    acceleration = ...
        (Fw - c*v - k*x)/meff;

    dydt = [v; acceleration];

end