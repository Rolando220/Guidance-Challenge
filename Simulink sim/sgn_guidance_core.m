function [acc, dbg] = sgn_guidance_core(mode, time_now, r_meas, sigma_meas, yaw, reset, params)
%SGN_GUIDANCE_CORE_CODEGEN Simulink/code-generation-safe guidance core.
%
% No evalin, no try/catch, no base-workspace access.
%
% mode:
%   1 -> Pursuit
%   2 -> PNG
%   3 -> APNG
%   4 -> Adaptive hybrid
%
% params is a 16x1 vector:
%   1  beta_memory
%   2  max_acc
%   3  kp_pursuit
%   4  kd_pursuit
%   5  N_png
%   6  png_pursuit_bias
%   7  N_apng
%   8  aT_perp_limit
%   9  tau_a
%   10 VC_MIN
%   11 R_TERMINAL
%   12 R_FAR
%   13 APNG_ON
%   14 APNG_OFF
%   15 terminal_pursuit_weight_maneuver
%   16 terminal_pursuit_weight_nominal
%
% dbg:
%   [time; r_meas; r_est; r_dot; sigma_meas; sigma_est;
%    sigma_dot; aT_perp; acc; mode_id]

%#codegen

persistent initialized
persistent t_old
persistent r_est
persistent r_dot_est
persistent sigma_est
persistent sigma_dot_est
persistent sigma_dot_old
persistent aT_perp_est
persistent mode_memory

% -------------------------------------------------------------------------
% Full persistent initialization for Simulink code generation
% -------------------------------------------------------------------------
if isempty(initialized)
    initialized = false;
end

if isempty(t_old)
    t_old = 0.0;
end

if isempty(r_est)
    r_est = 1e-3;
end

if isempty(r_dot_est)
    r_dot_est = 0.0;
end

if isempty(sigma_est)
    sigma_est = 0.0;
end

if isempty(sigma_dot_est)
    sigma_dot_est = 0.0;
end

if isempty(sigma_dot_old)
    sigma_dot_old = 0.0;
end

if isempty(aT_perp_est)
    aT_perp_est = 0.0;
end

if isempty(mode_memory)
    mode_memory = 2.0;
end

% -------------------------------------------------------------------------
% Parameter unpacking
% -------------------------------------------------------------------------
beta_memory = params(1);
max_acc     = params(2);

kp_pursuit = params(3);
kd_pursuit = params(4);

N_png   = params(5);
png_bias = params(6);

N_apng  = params(7);
aT_lim  = params(8);
tau_a   = params(9);

VC_MIN     = params(10);
R_TERMINAL = params(11);
R_FAR      = params(12);

APNG_ON  = params(13);
APNG_OFF = params(14);

w_terminal_maneuver = params(15);
w_terminal_nominal  = params(16);

% Safety clamps on parameters
if beta_memory < 0.05
    beta_memory = 0.05;
elseif beta_memory > 0.95
    beta_memory = 0.95;
end

if max_acc <= 0.0
    max_acc = 0.75;
end

if tau_a <= 1e-4
    tau_a = 1e-4;
end

% -------------------------------------------------------------------------
% Input conditioning
% -------------------------------------------------------------------------
r_meas = max(double(r_meas), 1e-3);
sigma_meas = sgn_wrap_pi(double(sigma_meas));
yaw = sgn_wrap_pi(double(yaw));
time_now = double(time_now);
mode = double(mode);

% -------------------------------------------------------------------------
% First-sample initialization / reset
% -------------------------------------------------------------------------
if (~initialized) || reset
    r_est = r_meas;
    r_dot_est = 0.0;

    sigma_est = sigma_meas;
    sigma_dot_est = 0.0;
    sigma_dot_old = 0.0;

    aT_perp_est = 0.0;
    t_old = time_now;

    mode_memory = 2.0;
    initialized = true;
end

dt = time_now - t_old;

% -------------------------------------------------------------------------
% 2nd-order fading filter for r and sigma
% -------------------------------------------------------------------------
if dt > 1e-5
    G = 1.0 - beta_memory^2;
    H = (1.0 - beta_memory)^2;

    % Range filter
    r_pred = r_est + dt*r_dot_est;
    r_dot_pred = r_dot_est;
    e_r = r_meas - r_pred;

    r_est = max(r_pred + G*e_r, 1e-3);     
    r_dot_est = r_dot_pred + H*e_r/dt; 

    % LOS filter
    sigma_pred = sgn_wrap_pi(sigma_est + dt*sigma_dot_est);
    sigma_dot_pred = sigma_dot_est;
    e_sigma = sgn_wrap_pi(sigma_meas - sigma_pred);

    sigma_est = sgn_wrap_pi(sigma_pred + G*e_sigma);
    sigma_dot_est = sigma_dot_pred + H*e_sigma/dt;

    % APNG maneuver indicator
    sigma_ddot_raw = (sigma_dot_est - sigma_dot_old)/dt;
    sigma_dot_old = sigma_dot_est;

    a_rel_perp_raw = r_est*sigma_ddot_raw + 2.0*r_dot_est*sigma_dot_est;

    alpha_a = tau_a/(tau_a + dt);
    aT_perp_est = alpha_a*aT_perp_est + (1.0 - alpha_a)*a_rel_perp_raw;

    t_old = time_now;
end

r = r_est;
r_dot = r_dot_est;
sigma = sigma_est;
sigma_dot = sigma_dot_est;
Vc = max(-r_dot, 0.0);

sigma_err = sgn_wrap_pi(yaw - sigma);

% -------------------------------------------------------------------------
% Individual laws
% -------------------------------------------------------------------------

% 1) Pursuit
acc_pursuit = -kp_pursuit*sigma_err - kd_pursuit*sigma_dot;

% 2) PNG
acc_png = N_png*Vc*sigma_dot - png_bias*sigma_err;

% 3) APNG
aT_perp = sgn_saturate(aT_perp_est, aT_lim);
acc_apng = N_apng*Vc*sigma_dot + 0.5*N_apng*aT_perp - 0.20*sigma_err;

% -------------------------------------------------------------------------
% Selection / adaptive hybrid
% -------------------------------------------------------------------------
mode_id = mode;
% 
% if mode == 1
%     acc = acc_pursuit;
% 
% elseif mode == 2
%     if Vc < VC_MIN
%         acc = acc_pursuit;
%         mode_id = 1.0;
%     else
%         acc = acc_png;
%     end
% 
% elseif mode == 3
%     if Vc < VC_MIN
%         acc = acc_pursuit;
%         mode_id = 1.0;
%     else
%         acc = acc_apng;
%     end
% 
% else
%     maneuver_level = abs(aT_perp);
% 
%     if mode_memory == 3.0
%         target_maneuvering = maneuver_level > APNG_OFF;
%     else
%         target_maneuvering = maneuver_level > APNG_ON;
%     end
% 
%     if Vc < VC_MIN
%         acc = acc_pursuit;
%         mode_id = 1.0;
% 
%     elseif r < R_TERMINAL
%         if target_maneuvering
%             acc = w_terminal_maneuver*acc_pursuit + ...
%                   (1.0 - w_terminal_maneuver)*acc_apng;
%         else
%             acc = w_terminal_nominal*acc_pursuit + ...
%                   (1.0 - w_terminal_nominal)*acc_png;
%         end
%         mode_id = 4.0;
% 
%     elseif target_maneuvering
%         acc = acc_apng;
%         mode_id = 3.0;
% 
%     elseif r > R_FAR
%         acc = 0.25*acc_pursuit + 0.75*acc_png;
%         mode_id = 4.0;
% 
%     else
%         acc = acc_png;
%         mode_id = 2.0;
%     end
% 
%     if mode_id == 1.0 || mode_id == 2.0 || mode_id == 3.0
%         mode_memory = mode_id;
%     end
% end

acc = acc_png;   %% se va si riaggiungerà il codice commentato
acc = sgn_saturate(acc, max_acc);

% -------------------------------------------------------------------------
% Debug vector
% -------------------------------------------------------------------------
dbg = zeros(10,1);
dbg(1)  = time_now;
dbg(2)  = r_meas;
dbg(3)  = r;
dbg(4)  = r_dot;
dbg(5)  = sigma_meas;
dbg(6)  = sigma;
dbg(7)  = sigma_dot;
dbg(8)  = aT_perp;
dbg(9)  = acc;
dbg(10) = mode_id;

end
