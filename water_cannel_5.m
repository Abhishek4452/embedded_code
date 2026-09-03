%% ========================================================================
%  VERTICALLY CONSTRAINED FLOATING BALL IN A LOW-FLOW WATER CANAL
%
%  SINGLE-DEGREE-OF-FREEDOM VERTICAL FLUID-STRUCTURE INTERACTION MODEL
%
%  Governing equation:
%
%       meff*x_ddot + c*x_dot + k*x = Fw(t)
%
%  where:
%
%       x       = vertical displacement from equilibrium [m]
%       x_dot   = vertical velocity [m/s]
%       x_ddot  = vertical acceleration [m/s^2]
%       meff    = effective moving mass [kg]
%       c       = equivalent damping [N.s/m]
%       k       = equivalent vertical stiffness [N/m]
%       Fw(t)   = fluctuating vertical hydrodynamic force [N]
%
%
%  Hydrodynamic excitation:
%
%       Fw(t) = Aw*sin(2*pi*fw*t)
%             + Av*sin(2*pi*fv*t)
%             + Fn(t)
%
%  where:
%
%       Aw = water-surface excitation amplitude
%       fw = water-surface disturbance frequency
%       Av = vortex excitation amplitude
%       fv = vortex shedding frequency
%       Fn = filtered stochastic turbulence force
%
%
%  PHYSICAL INTERPRETATION
%  -----------------------
%
%  1. The ball is horizontally constrained.
%     Therefore:
%
%           x_horizontal = 0
%
%     and only vertical motion is allowed.
%
%  2. Gravity and static buoyancy are assumed to balance at the
%     equilibrium position. Therefore they do not explicitly appear
%     in the dynamic equation.
%
%  3. The spring represents an equivalent vertical restoring stiffness.
%     It can represent the combined effect of:
%          - tether/support stiffness
%          - effective buoyancy restoring force
%          - structural restoring effects
%
%  4. The damper represents energy dissipation due to:
%          - water drag
%          - material/support damping
%          - other dissipative mechanisms
%
%  5. Mean water velocity DOES NOT directly create a constant vibration.
%     Instead, U is used to estimate the vortex-shedding frequency.
%
%  6. Turbulence is represented by filtered random noise.
%     This is an engineering approximation and is NOT a CFD model.
%
%  7. Water-surface disturbance is represented by a sinusoidal
%     disturbance. Real canals will have more complicated wave motion.
%
%
%  IMPORTANT:
%  ----------
%  This is a reduced-order engineering model.
%  It is useful for studying vibration trends, RMS acceleration,
%  resonance and the effect of changing flow conditions.
%
%  It is NOT a full CFD fluid-structure interaction simulation.
%
% ========================================================================

clc;
clear;
close all;


%% ========================================================================
% 1. USER-ADJUSTABLE PARAMETERS
% ========================================================================

%% -------------------- BALL PARAMETERS -----------------------------------

% Physical mass of floating ball
m_ball = 0.030;                    % [kg]

% Approximate hydrodynamic added mass
m_added = 0.005;                   % [kg]

% Effective moving mass
meff = m_ball + m_added;            % [kg]

% Ball diameter
ball_diameter = 0.060;             % [m]

% Ball radius
ball_radius = ball_diameter/2;     % [m]


%% -------------------- VERTICAL CONSTRAINT -------------------------------

% Equivalent vertical stiffness
%
% Increasing k:
%       -> increases natural frequency
%       -> generally reduces displacement for a given force
%
k = 12.0;                           % [N/m]

% Equivalent damping
%
% Increasing c:
%       -> increases energy dissipation
%       -> reduces oscillation amplitude
%       -> generally reduces vibration RMS
%
c = 0.080;                          % [N.s/m]


%% -------------------- WATER PARAMETERS ----------------------------------

% Mean water velocity
%
% This does NOT directly generate continuous vibration.
% It is used to determine the vortex shedding frequency.
%
U = 0.25;                           % [m/s]

% Water density
rho = 1000;                         % [kg/m^3]


%% -------------------- WATER SURFACE DISTURBANCE --------------------------

% Water-surface force amplitude
Aw = 0.030;                         % [N]

% Water-surface disturbance frequency
fw = 0.15;                          % [Hz]

% Actual visible water-level disturbance
water_surface_amp = 0.004;         % [m]


%% -------------------- VORTEX SHEDDING -----------------------------------

% Strouhal number
%
% For bluff bodies, approximately 0.1-0.2 can be used as an
% engineering estimate.
%
St = 0.20;

% Vortex shedding frequency
%
%       fv = St*U/D
%
fv = St*U/ball_diameter;             % [Hz]

% Vortex excitation amplitude
Av = 0.020;                         % [N]


%% -------------------- TURBULENCE ----------------------------------------

% Desired RMS magnitude of turbulent force
Fn_rms = 0.008;                     % [N]

% Approximate turbulence cutoff frequency
%
% Larger value:
%       -> faster turbulent fluctuations
%
f_turb = 2.0;                       % [Hz]


%% -------------------- INITIAL TRANSIENT ---------------------------------

% Initial vertical displacement
%
% This gives the ball an initial disturbance.
%
initial_displacement = 0.020;      % [m]

% Initial vertical velocity
initial_velocity = 0.000;           % [m/s]


%% -------------------- SIMULATION TIME -----------------------------------

% Duration of natural transient response
T_transient = 8;                    % [s]

% Duration of sustained hydrodynamic excitation
T_sustained = 60;                   % [s]

% Total simulation time
T_total = T_transient + T_sustained;


%% -------------------- NUMERICAL SAMPLING --------------------------------

% Sampling frequency
Fs = 200;                           % [Hz]

% Sampling interval
dt = 1/Fs;                          % [s]

% Transient time vector
t1 = (0:dt:T_transient).';

% Sustained excitation time vector
t2 = (T_transient+dt:dt:T_total).';


%% -------------------- RUNNING RMS WINDOW --------------------------------

% Window length for moving RMS
RMS_window_seconds = 2.0;           % [s]

% Number of samples in RMS window
RMS_window_samples = round( ...
    RMS_window_seconds*Fs);


%% -------------------- RANDOM TURBULENCE SEED -----------------------------

% Fix random seed so the same simulation can be reproduced.
rng(10);


%% ========================================================================
% 2. SYSTEM NATURAL FREQUENCY
% ========================================================================

% Undamped natural angular frequency
wn_natural = sqrt(k/meff);           % [rad/s]

% Undamped natural frequency
fn_natural = wn_natural/(2*pi);      % [Hz]

% Damping ratio
zeta = c/(2*sqrt(k*meff));

% Damped natural frequency
if zeta < 1
    fd_natural = fn_natural*sqrt(1-zeta^2);
else
    fd_natural = 0;
end


%% ========================================================================
% 3. GENERATE FILTERED STOCHASTIC TURBULENCE
% ========================================================================
%
% Raw Gaussian random noise is generated first.
%
% Then a first-order low-pass filter is used.
%
% This represents the fact that real turbulence has a finite
% correlation time rather than infinitely fast fluctuations.
%
% This is an engineering approximation.
%
% ========================================================================

N2 = length(t2);

% Generate Gaussian random noise
raw_noise = randn(N2,1);

% Turbulence correlation time
tau_turb = 1/(2*pi*f_turb);

% First-order filter coefficient
alpha = dt/(tau_turb + dt);

% Allocate filtered noise
filtered_noise = zeros(N2,1);

% Apply simple low-pass filter
for i = 2:N2

    filtered_noise(i) = ...
        filtered_noise(i-1) + ...
        alpha*(raw_noise(i)-filtered_noise(i-1));

end


% Normalize filtered noise to desired RMS
current_rms = sqrt(mean(filtered_noise.^2));

if current_rms > 0

    filtered_noise = ...
        filtered_noise/current_rms * Fn_rms;

end


% Turbulent force
Fn = filtered_noise;


%% ========================================================================
% 4. GENERATE SUSTAINED HYDRODYNAMIC EXCITATION
% ========================================================================

% Time measured from beginning of sustained excitation
t_forcing = t2 - T_transient;


% -------------------- WATER SURFACE COMPONENT ----------------------------

Fw_surface = ...
    Aw*sin(2*pi*fw*t_forcing);


% -------------------- VORTEX COMPONENT -----------------------------------

Fw_vortex = ...
    Av*sin(2*pi*fv*t_forcing);


% -------------------- TURBULENCE COMPONENT -------------------------------

Fw_turb = Fn;


% -------------------- TOTAL FORCE ----------------------------------------

Fw2 = ...
    Fw_surface + ...
    Fw_vortex + ...
    Fw_turb;


%% ========================================================================
% 5. INITIAL TRANSIENT PHASE
% ========================================================================
%
% During the initial transient:
%
%       Fw(t) = 0
%
% The ball starts with an initial displacement/velocity and then
% oscillates naturally.
%
% Damping causes the oscillation to decay.
%
% ========================================================================

% Initial state
x0 = [initial_displacement;
      initial_velocity];


% ODE solver options
ode_options = odeset( ...
    'RelTol',1e-7, ...
    'AbsTol',1e-9);


% Solve free vibration using ode45
[t1_sol,y1] = ode45( ...
    @(tt,yy) vertical_ball_ode( ...
        tt,yy,meff,c,k,@(time) 0), ...
    t1, ...
    x0, ...
    ode_options);


% Extract displacement
x1 = y1(:,1);


% Extract velocity
v1 = y1(:,2);


% Calculate acceleration
a1 = zeros(size(x1));

for i = 1:length(x1)

    Fcurrent = 0;

    a1(i) = ...
        (Fcurrent - c*v1(i) - k*x1(i))/meff;

end


%% ========================================================================
% 6. SUSTAINED EXCITATION PHASE
% ========================================================================
%
% Now the water produces:
%
%       water surface disturbance
%       vortex shedding
%       stochastic turbulence
%
% The ODE solver calculates the resulting ball motion.
%
% ========================================================================


% Create an absolute-time force vector.
%
% Include t = T_transient explicitly so that interpolation is
% well-defined at the phase transition.
%
t_force_absolute = ...
    [T_transient;
     t2];


% Force is zero exactly at the transition and then follows Fw2.
Fw_absolute = ...
    [0;
     Fw2];


% Interpolation function for hydrodynamic force
force_function = @(tt) ...
    interp1( ...
        t_force_absolute, ...
        Fw_absolute, ...
        tt, ...
        'linear', ...
        'extrap');


% Initial state for sustained phase
x0_forced = ...
    [x1(end);
     v1(end)];


% Solve forced response using ode45
[t2_sol,y2] = ode45( ...
    @(tt,yy) vertical_ball_ode( ...
        tt,yy,meff,c,k,force_function), ...
    t2, ...
    x0_forced, ...
    ode_options);


% Extract displacement
x2 = y2(:,1);


% Extract velocity
v2 = y2(:,2);


% Calculate acceleration
a2 = zeros(size(x2));

for i = 1:length(x2)

    Fcurrent = force_function(t2_sol(i));

    a2(i) = ...
        (Fcurrent - c*v2(i) - k*x2(i))/meff;

end


%% ========================================================================
% 7. COMBINE TRANSIENT + SUSTAINED PHASE
% ========================================================================

% Combine time
t = [t1_sol;
     t2_sol];


% Combine displacement
x = [x1;
     x2];


% Combine velocity
v = [v1;
     v2];


% Combine acceleration
a = [a1;
     a2];


% Combine total force
Fw_total = ...
    [zeros(size(t1_sol));
     Fw2];


%% ========================================================================
% 8. CALCULATE WATER LEVEL
% ========================================================================
%
% This is only the visible water-level variation.
%
% It is not directly substituted into the dynamic equation.
% Its hydrodynamic effect is represented through Fw_surface.
%
% ========================================================================

water_level = zeros(size(t));


% Transient phase
water_level(1:length(t1_sol)) = 0;


% Sustained phase
water_level(length(t1_sol)+1:end) = ...
    water_surface_amp * ...
    sin(2*pi*fw*t_forcing);


%% ========================================================================
% 9. ACCELERATION RMS
% ========================================================================
%
% Definition:
%
%              N
%             ----
%       RMS = sqrt( 1/N * SUM(ai^2) )
%             ----
%              i=1
%
% ========================================================================

% Overall acceleration RMS
accel_RMS = ...
    sqrt(mean(a.^2));


% Sustained-phase acceleration RMS
accel_RMS_sustained = ...
    sqrt(mean(a2.^2));


%% ========================================================================
% 10. MANUAL MOVING / RUNNING RMS
% ========================================================================
%
% We DO NOT use movrms().
%
% This implementation works without movrms().
%
% For every point in time, a window around that point is selected
% and RMS acceleration is calculated.
%
% ========================================================================

running_RMS = zeros(size(a));

% Half window
half_window = ...
    floor(RMS_window_samples/2);


for i = 1:length(a)

    % Beginning of window
    start_index = ...
        max(1,i-half_window);

    % End of window
    end_index = ...
        min(length(a),i+half_window);

    % Extract acceleration window
    window_data = ...
        a(start_index:end_index);

    % Calculate RMS
    running_RMS(i) = ...
        sqrt(mean(window_data.^2));

end


%% ========================================================================
% 11. PEAK ACCELERATION
% ========================================================================

peak_acceleration = ...
    max(abs(a));


%% ========================================================================
% 12. PEAK-TO-PEAK ACCELERATION
% ========================================================================

peak_to_peak_acceleration = ...
    max(a) - min(a);


%% ========================================================================
% 13. FFT OF SUSTAINED ACCELERATION
% ========================================================================
%
% FFT is calculated using the sustained response because this section
% represents the long-term vibration caused by the flowing water.
%
% ========================================================================

Nfft = length(a2);


% Remove DC component
a_fft_signal = ...
    a2 - mean(a2);


% FFT
Y = fft(a_fft_signal);


% Two-sided amplitude spectrum
P2 = abs(Y/Nfft);


% One-sided spectrum
P1 = ...
    P2(1:floor(Nfft/2)+1);


% Correct amplitude
if length(P1) > 2

    P1(2:end-1) = ...
        2*P1(2:end-1);

end


% Frequency vector
f_fft = ...
    Fs*(0:floor(Nfft/2))/Nfft;


%% ========================================================================
% 14. DOMINANT VIBRATION FREQUENCY
% ========================================================================

% Ignore DC component

if length(P1) > 1

    [~,idx_dom] = ...
        max(P1(2:end));

    idx_dom = ...
        idx_dom + 1;

else

    idx_dom = 1;

end


dominant_frequency = ...
    f_fft(idx_dom);


%% ========================================================================
% 15. POWER SPECTRAL DENSITY
% ========================================================================
%
% Manual periodogram calculation.
%
% This avoids requiring pwelch() from the Signal Processing Toolbox.
%
% ========================================================================

PSD_two_sided = ...
    abs(Y).^2/(Fs*Nfft);


% One-sided PSD
PSD_one_sided = ...
    PSD_two_sided(1:floor(Nfft/2)+1);


% Correct one-sided PSD
if length(PSD_one_sided) > 2

    PSD_one_sided(2:end-1) = ...
        2*PSD_one_sided(2:end-1);

end


f_psd = f_fft;


%% ========================================================================
% 16. COMMAND-WINDOW SUMMARY
% ========================================================================

fprintf('\n');
fprintf('==============================================================\n');
fprintf('      VERTICAL FLOATING BALL SIMULATION SUMMARY\n');
fprintf('==============================================================\n');

fprintf('\n');

fprintf('BALL PARAMETERS\n');
fprintf('Ball mass                    : %.5f kg\n',m_ball);
fprintf('Added hydrodynamic mass      : %.5f kg\n',m_added);
fprintf('Effective mass               : %.5f kg\n',meff);
fprintf('Ball diameter                : %.5f m\n',ball_diameter);

fprintf('\n');

fprintf('MECHANICAL PARAMETERS\n');
fprintf('Equivalent stiffness         : %.5f N/m\n',k);
fprintf('Damping coefficient          : %.5f N.s/m\n',c);
fprintf('Damping ratio                : %.5f\n',zeta);

fprintf('\n');

fprintf('WATER PARAMETERS\n');
fprintf('Mean water velocity          : %.5f m/s\n',U);
fprintf('Water density                : %.1f kg/m^3\n',rho);

fprintf('\n');

fprintf('EXCITATION PARAMETERS\n');
fprintf('Water disturbance frequency  : %.5f Hz\n',fw);
fprintf('Vortex shedding frequency    : %.5f Hz\n',fv);
fprintf('Turbulence RMS force         : %.5f N\n',Fn_rms);

fprintf('\n');

fprintf('DYNAMIC RESULTS\n');
fprintf('Natural frequency             : %.5f Hz\n',fn_natural);
fprintf('Damped natural frequency      : %.5f Hz\n',fd_natural);
fprintf('Acceleration RMS              : %.6f m/s^2\n',accel_RMS);
fprintf('Sustained acceleration RMS    : %.6f m/s^2\n',accel_RMS_sustained);
fprintf('Peak acceleration             : %.6f m/s^2\n',peak_acceleration);
fprintf('Peak-to-peak acceleration     : %.6f m/s^2\n', ...
    peak_to_peak_acceleration);
fprintf('Dominant vibration frequency  : %.5f Hz\n',dominant_frequency);

fprintf('\n');

fprintf('==============================================================\n');
fprintf('Transient duration            : %.2f s\n',T_transient);
fprintf('Sustained excitation duration : %.2f s\n',T_sustained);
fprintf('Total simulation time         : %.2f s\n',T_total);
fprintf('==============================================================\n');

fprintf('\n');


%% ========================================================================
% 17. CREATE MAIN FIGURE
% ========================================================================
%
% Everything is placed into ONE figure using tiledlayout.
%
% 1. Virtual canal
% 2. Water disturbance
% 3. Hydrodynamic force
% 4. Vertical displacement
% 5. Vertical velocity
% 6. Vertical acceleration
% 7. Running acceleration RMS
% 8. FFT / frequency spectrum
%
% ========================================================================

fig = figure( ...
    'Name','Vertical Floating Ball - Water Canal Simulation', ...
    'Color','w', ...
    'Position',[20 30 1550 900]);


% 4 rows x 2 columns
tl = tiledlayout(fig,4,2, ...
    'TileSpacing','compact', ...
    'Padding','compact');


% Overall title
sgtitle(tl, ...
    'Vertically Constrained Floating Ball in a Low-Flow Water Canal', ...
    'FontSize',16, ...
    'FontWeight','bold');


%% ========================================================================
% TILE 1
% VIRTUAL CANAL + BALL
% ========================================================================

ax1 = nexttile(tl,1);

hold(ax1,'on');
grid(ax1,'on');


% -------------------- CANAL GEOMETRY -------------------------------------

canal_width = 1.20;
canal_bottom = 0;
canal_top = 0.85;


% Nominal water level
water_nominal = 0.48;


% Canal bottom
plot(ax1, ...
    [-canal_width/2 canal_width/2], ...
    [canal_bottom canal_bottom], ...
    'k', ...
    'LineWidth',3);


% Left wall
plot(ax1, ...
    [-canal_width/2 -canal_width/2], ...
    [canal_bottom canal_top], ...
    'k', ...
    'LineWidth',3);


% Right wall
plot(ax1, ...
    [canal_width/2 canal_width/2], ...
    [canal_bottom canal_top], ...
    'k', ...
    'LineWidth',3);


% -------------------- WATER PATCH ---------------------------------------

water_x = ...
    [-canal_width/2 ...
      canal_width/2 ...
      canal_width/2 ...
     -canal_width/2];


water_y = ...
    [water_nominal ...
     water_nominal ...
     canal_bottom ...
     canal_bottom];


water_patch = patch( ...
    ax1, ...
    water_x, ...
    water_y, ...
    [0.70 0.88 1.00], ...
    'EdgeColor','none');


% -------------------- WATER SURFACE -------------------------------------

water_line = plot(ax1, ...
    [-canal_width/2 canal_width/2], ...
    [water_nominal water_nominal], ...
    'b', ...
    'LineWidth',2);


% -------------------- OVERHEAD SUPPORT ----------------------------------

support_y = 0.78;


plot(ax1, ...
    [-0.18 0.18], ...
    [support_y support_y], ...
    'k', ...
    'LineWidth',6);


% -------------------- VERTICAL GUIDE ------------------------------------

plot(ax1, ...
    [0 0], ...
    [support_y-0.02 support_y-0.10], ...
    'k--', ...
    'LineWidth',1);


% -------------------- BALL EQUILIBRIUM ----------------------------------

ball_eq_y = ...
    water_nominal + 0.010;


% Ball
ball_plot = plot(ax1, ...
    0, ...
    ball_eq_y, ...
    'o', ...
    'MarkerSize',20, ...
    'MarkerFaceColor',[1.0 0.50 0.10], ...
    'MarkerEdgeColor','k', ...
    'LineWidth',1.5);


% -------------------- TETHER / SPRING -----------------------------------

spring_line = plot(ax1, ...
    [0 0], ...
    [support_y ball_eq_y], ...
    'k-', ...
    'LineWidth',1.5);


% -------------------- HORIZONTAL CONSTRAINT -----------------------------

plot(ax1, ...
    [-0.10 0.10], ...
    [ball_eq_y ball_eq_y], ...
    'k:', ...
    'LineWidth',1);


% Labels
xlabel(ax1,'Horizontal position [m]');
ylabel(ax1,'Vertical position [m]');

title(ax1,'Virtual Water Canal + Vertically Constrained Ball');


% Axis limits
xlim(ax1,[-canal_width/2 canal_width/2]);
ylim(ax1,[canal_bottom canal_top]);


% Keep aspect reasonable
axis(ax1,'manual');


% Information text
text(ax1, ...
    -0.55, ...
    0.72, ...
    sprintf('Mean velocity = %.2f m/s',U), ...
    'FontSize',9);


text(ax1, ...
    -0.55, ...
    0.67, ...
    'Horizontal displacement = 0', ...
    'FontSize',9);


text(ax1, ...
    -0.55, ...
    0.62, ...
    'Only vertical motion allowed', ...
    'FontSize',9);


%% ========================================================================
% TILE 2
% WATER LEVEL DISTURBANCE
% ========================================================================

ax2 = nexttile(tl,2);

plot(ax2, ...
    t, ...
    water_level*1000, ...
    'b', ...
    'LineWidth',1.3);

hold(ax2,'on');


xline(ax2, ...
    T_transient, ...
    'k--', ...
    'Transient / forced');


xlabel(ax2,'Time [s]');
ylabel(ax2,'Water-level disturbance [mm]');

title(ax2,'Water-Surface Disturbance');

grid(ax2,'on');

xlim(ax2,[0 T_total]);


%% ========================================================================
% TILE 3
% HYDRODYNAMIC FORCE
% ========================================================================

ax3 = nexttile(tl,3);


% Total force
plot(ax3, ...
    t, ...
    Fw_total, ...
    'LineWidth',1.2);

hold(ax3,'on');


% Water surface force
plot(ax3, ...
    t2, ...
    Fw_surface, ...
    '--', ...
    'LineWidth',0.9);


% Vortex force
plot(ax3, ...
    t2, ...
    Fw_vortex, ...
    '--', ...
    'LineWidth',0.9);


% Turbulent force
plot(ax3, ...
    t2, ...
    Fw_turb, ...
    ':', ...
    'LineWidth',0.9);


xline(ax3, ...
    T_transient, ...
    'k--', ...
    'Transient / forced');


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


%% ========================================================================
% TILE 4
% VERTICAL DISPLACEMENT
% ========================================================================

ax4 = nexttile(tl,4);


plot(ax4, ...
    t, ...
    x*1000, ...
    'LineWidth',1.3);

hold(ax4,'on');


xline(ax4, ...
    T_transient, ...
    'k--', ...
    'Transient / forced');


xlabel(ax4,'Time [s]');
ylabel(ax4,'Vertical displacement [mm]');

title(ax4,'Vertical Displacement');

grid(ax4,'on');

xlim(ax4,[0 T_total]);


%% ========================================================================
% TILE 5
% VERTICAL VELOCITY
% ========================================================================

ax5 = nexttile(tl,5);


plot(ax5, ...
    t, ...
    v, ...
    'LineWidth',1.2);

hold(ax5,'on');


xline(ax5, ...
    T_transient, ...
    'k--', ...
    'Transient / forced');


xlabel(ax5,'Time [s]');
ylabel(ax5,'Vertical velocity [m/s]');

title(ax5,'Vertical Velocity');

grid(ax5,'on');

xlim(ax5,[0 T_total]);


%% ========================================================================
% TILE 6
% VERTICAL ACCELERATION
% ========================================================================

ax6 = nexttile(tl,6);


plot(ax6, ...
    t, ...
    a, ...
    'LineWidth',1.1);

hold(ax6,'on');


xline(ax6, ...
    T_transient, ...
    'k--', ...
    'Transient / forced');


xlabel(ax6,'Time [s]');
ylabel(ax6,'Acceleration [m/s^2]');

title(ax6,'Vertical Acceleration');

grid(ax6,'on');

xlim(ax6,[0 T_total]);


% RMS annotation
text(ax6, ...
    0.02*T_total, ...
    0.85*max(a), ...
    sprintf('RMS = %.4f m/s^2',accel_RMS), ...
    'FontWeight','bold');


% Peak annotation
text(ax6, ...
    0.02*T_total, ...
    0.65*max(a), ...
    sprintf('Peak = %.4f m/s^2',peak_acceleration), ...
    'FontWeight','bold');


%% ========================================================================
% TILE 7
% RUNNING ACCELERATION RMS
% ========================================================================

ax7 = nexttile(tl,7);


plot(ax7, ...
    t, ...
    running_RMS, ...
    'LineWidth',1.4);

hold(ax7,'on');


xline(ax7, ...
    T_transient, ...
    'k--', ...
    'Transient / forced');


yline(ax7, ...
    accel_RMS_sustained, ...
    'r--', ...
    sprintf('Sustained RMS = %.4f', ...
    accel_RMS_sustained));


xlabel(ax7,'Time [s]');
ylabel(ax7,'Acceleration RMS [m/s^2]');


title(ax7, ...
    sprintf('Running Acceleration RMS (%.1f s window)', ...
    RMS_window_seconds));


grid(ax7,'on');

xlim(ax7,[0 T_total]);


%% ========================================================================
% TILE 8
% FFT / FREQUENCY SPECTRUM
% ========================================================================

ax8 = nexttile(tl,8);


plot(ax8, ...
    f_fft, ...
    P1, ...
    'LineWidth',1.2);

hold(ax8,'on');


% Natural frequency
xline(ax8, ...
    fn_natural, ...
    'k--', ...
    sprintf('Natural = %.2f Hz',fn_natural));


% Vortex frequency
xline(ax8, ...
    fv, ...
    'r--', ...
    sprintf('Vortex = %.2f Hz',fv));


% Water frequency
xline(ax8, ...
    fw, ...
    'b--', ...
    sprintf('Water = %.2f Hz',fw));


% Dominant frequency
xline(ax8, ...
    dominant_frequency, ...
    'm-', ...
    sprintf('Dominant = %.2f Hz',dominant_frequency), ...
    'LineWidth',1.5);


xlabel(ax8,'Frequency [Hz]');
ylabel(ax8,'Acceleration amplitude [m/s^2]');


title(ax8,'Acceleration FFT / Frequency Spectrum');


grid(ax8,'on');


% Display first 10 Hz
xlim(ax8,[0 min(10,Fs/2)]);


%% ========================================================================
% 18. ADD SUMMARY ANNOTATION TO FIGURE
% ========================================================================

annotation(fig, ...
    'textbox', ...
    [0.70 0.945 0.28 0.035], ...
    'String', ...
    sprintf( ...
    'RMS = %.4f m/s^2   |   Peak = %.4f m/s^2   |   f_n = %.2f Hz   |   f_{dom} = %.2f Hz', ...
    accel_RMS, ...
    peak_acceleration, ...
    fn_natural, ...
    dominant_frequency), ...
    'FitBoxToText','off', ...
    'HorizontalAlignment','center', ...
    'FontWeight','bold', ...
    'EdgeColor','k');


%% ========================================================================
% 19. ANIMATION
% ========================================================================
%
% The animation uses the calculated displacement:
%
%       y_ball(t) = y_equilibrium + x(t)
%
% The horizontal coordinate remains exactly zero.
%
% Therefore the ball visibly moves ONLY vertically.
%
% The water surface also changes according to the simulated
% low-frequency disturbance.
%
% ========================================================================

fprintf('\nStarting virtual canal animation...\n');
fprintf('Close the figure window to stop the animation.\n\n');


% Approximate animation rate
animation_skip = ...
    max(1,round(Fs/25));


for i = 1:animation_skip:length(t)


    % --------------------------------------------------------------
    % Current ball vertical position
    % --------------------------------------------------------------

    ball_y = ...
        ball_eq_y + x(i);


    % --------------------------------------------------------------
    % Current water level
    % --------------------------------------------------------------

    current_water = ...
        water_nominal + water_level(i);


    % --------------------------------------------------------------
    % Update water patch
    % --------------------------------------------------------------

    water_patch.XData = ...
        [-canal_width/2 ...
          canal_width/2 ...
          canal_width/2 ...
         -canal_width/2];


    water_patch.YData = ...
        [current_water ...
         current_water ...
         canal_bottom ...
         canal_bottom];


    % --------------------------------------------------------------
    % Update water surface
    % --------------------------------------------------------------

    water_line.YData = ...
        [current_water current_water];


    % --------------------------------------------------------------
    % Update ball
    % --------------------------------------------------------------

    ball_plot.XData = 0;

    ball_plot.YData = ...
        ball_y;


    % --------------------------------------------------------------
    % Update vertical tether
    % --------------------------------------------------------------

    spring_line.XData = ...
        [0 0];

    spring_line.YData = ...
        [support_y ball_y];


    % --------------------------------------------------------------
    % Update title
    % --------------------------------------------------------------

    title(ax1, ...
        sprintf( ...
        'Virtual Canal - Time = %.2f s', ...
        t(i)));


    % --------------------------------------------------------------
    % Update animation
    % --------------------------------------------------------------

    drawnow limitrate;

end


fprintf('Animation complete.\n');


%% ========================================================================
% 20. SHOW FINAL BALL POSITION
% ========================================================================

final_ball_y = ...
    ball_eq_y + x(end);


final_water = ...
    water_nominal + water_level(end);


ball_plot.XData = 0;

ball_plot.YData = ...
    final_ball_y;


water_line.YData = ...
    [final_water final_water];


water_patch.YData = ...
    [final_water ...
     final_water ...
     canal_bottom ...
     canal_bottom];


spring_line.YData = ...
    [support_y final_ball_y];


drawnow;


%% ========================================================================
% 21. END OF MAIN SCRIPT
% ========================================================================


fprintf('\n');
fprintf('Simulation finished successfully.\n');
fprintf('\n');


%% ========================================================================
% LOCAL FUNCTION
% ========================================================================
%
% This function represents the equation:
%
%       meff*x_ddot + c*x_dot + k*x = Fw(t)
%
%
% Rearranging:
%
%       x_ddot = [Fw(t) - c*x_dot - k*x] / meff
%
% State vector:
%
%       y(1) = x
%       y(2) = x_dot
%
% Therefore:
%
%       y_dot(1) = x_dot
%       y_dot(2) = x_ddot
%
% ========================================================================

function dydt = vertical_ball_ode(t,y,meff,c,k,force_function)

    % Vertical displacement
    x = y(1);

    % Vertical velocity
    v = y(2);

    % Current hydrodynamic force
    Fw = force_function(t);

    % Governing dynamic equation
    acceleration = ...
        (Fw - c*v - k*x)/meff;

    % State derivatives
    dydt = ...
        [v;
         acceleration];

end