%% SGN_drone_challenge_ini_codegen_safe.m
% Parameters for code-generation-safe MATLAB Function block.
% No evalin is used by the guidance: all parameters are passed through
% SGN_GUIDANCE_PARAMS.

clearvars -except ans
clc
Tautopilot = 0.35;
%% Initial conditions from Python challenge file
XM_INI = -1.0;
YM_INI = -1.0;

heading_error_deg = 0;
GAMMA_INI = (90 - heading_error_deg)*pi/180;
VM = 0.25;

VXM_INI = VM*cos(GAMMA_INI);
VYM_INI = VM*sin(GAMMA_INI);

XT_INI = 1.0;
YT_INI = 0.0;

BETA_INI = 225*pi/180;
VT = 0.2;

LOS_INI = atan2(YT_INI - YM_INI, XT_INI - XM_INI);

%% Challenge limits
MAX_ACC = 0.25;
GUIDE_SEL = 0;
RTOL = 0.01;
STOP_TIME = 25;

ROOM_X_MIN = -1.4;
ROOM_X_MAX =  1.3;
ROOM_Y_MIN = -1.4;
ROOM_Y_MAX =  1.7;

% %% Target switching
% TARGET_MODE = 1;
% T1_SWITCH = 5.0;
% T2_SWITCH = 10.0;
% AT_PHASE1 = -0.075;
% AT_PHASE2 =  0.200;
% AT_PHASE3 = -0.250;
% AT = AT_PHASE1;

%% Guidance parameters
beta_memory = 0.55;
max_acc = MAX_ACC;

kp_pursuit = 1.70;
kd_pursuit = 0.08;

N_png = 4.20;
png_pursuit_bias = 0.25;

N_apng = 4.80;
aT_perp_limit = 0.35;
tau_a = 0.35;

VC_MIN = 0.03;
R_TERMINAL = 0.20;
R_FAR = 0.60;

APNG_ON = 0.10;
APNG_OFF = 0.05;

terminal_pursuit_weight_maneuver = 0.60;
terminal_pursuit_weight_nominal = 0.70;

SGN_GUIDANCE_PARAMS = [
    beta_memory;
    max_acc;
    kp_pursuit;
    kd_pursuit;
    N_png;
    png_pursuit_bias;
    N_apng;
    aT_perp_limit;
    tau_a;
    VC_MIN;
    R_TERMINAL;
    R_FAR;
    APNG_ON;
    APNG_OFF;
    terminal_pursuit_weight_maneuver;
    terminal_pursuit_weight_nominal
];

GUIDANCE_MODE = 2;  % 1 pursuit, 2 PNG, 3 APNG, 4 adaptive

fprintf('\nSGN guidance parameters loaded.\n');
fprintf('Use SGN_GUIDANCE_PARAMS as Constant block input to Guidance MF block.\n\n');
