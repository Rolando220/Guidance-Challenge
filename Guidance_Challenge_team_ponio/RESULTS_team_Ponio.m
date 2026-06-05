clear
close all

%% 20/05/2026
%name = "INSERIRE NOME.txt"; % 
%name = "logging_script__20260521_173010.txt"%primo volo 21/05:  Pure Pursuit -> non funziona, il target esce fuori dall'area e il drone rientra a casa
%name = "logging_script__20260521_173806.txt" %seconda prova 21/05: Pure Pursuit -> funziona e lo becca
%name = "logging_script__20260521_174852.txt" % Terza prova: Nostra guida png, accelerazione forse toppo alta... primo schianto e sono pure sminchiati i log (Non male)
%name = "logging_script__20260521_182706.txt" % test con pursuit loro per vedere se non si era scassato
name = "logging_script__20260527_140638.txt"; % test con derivata e variabili persistenti prova pursuit
name = "logging_script__20260527_145534.txt"; % test con fading filter e variabili persistenti prova pursuit GRAFICI TOP FILTRO
name = "logging_script__20260527_151332.txt"; % test con fading filter e PNG nostra GRAFICI INCREDIBILI
name = "logging_script__20260527_151945.txt" % PARALLELI png nostra primo try
name = "logging_script__20260527_152626.txt" % SGHEMBI png nostra 
name = "logging_script__20260527_153843.txt" % PARALLELI png con bias (TOP)
name = "logging_script__20260527_154143.txt" % SGHEMBI png con bias (TOPPERIA MISSILE)
name = "logging_script__20260527_154509.txt" % OPPOSTI png con bias (TOPPERIA INCREDIBILE MISSILISTICA)
name = "logging_script__20260527_155520.txt" % TARGET ACCELERANTE PNG (fuori dal box, segni accelerazioni sbagliati)
name = "logging_script__20260527_155840.txt" % Target accelerante no switch png
name = "logging_script__20260527_160132.txt" % Target accelerante con switch verso il drone png
name = "logging_script__20260527_160625.txt" % target accelerante no switch
name = "logging_script__20260527_160906.txt" % Target accelerante con switch fatto meglio
name = "logging_script__20260527_161606.txt" % Target accelerante con 2 switch (BELLO)
name = "logging_script__20260527_161908.txt" % Target intelligente loro (Out of box)
name = "logging_script__20260527_162334.txt" % Target intelligente loro (colpito)
name = "logging_script__20260527_162638.txt"
name = "logging_script__20260529_141321.txt" % test 29/05: target intelligente ZEM
name = "logging_script__20260529_150131.txt" 
name = "logging_script__20260529_150841.txt" % target intelligente ZEM, heading 110 target, guida fallisce
name = "logging_script__20260529_152535.txt" % come prima ma dopo la calibrazione
name = "logging_script__20260529_153153.txt"
name = "logging_script__20260529_153558.txt"
name = "logging_script__20260529_154113.txt" % log figo FN5S
name = "logging_script__20260529_154507.txt" % a scuola prendevo spesso 4.5
name = "logging_script__20260529_155317.txt" % biscotto pesa 0.35
name = "logging_script__20260529_155946.txt" % retry
name = "logging_script__20260529_161413.txt" % guida adattiva
name = "logging_script__20260529_161756.txt" % cambio parametri
name = "logging_script__20260529_162256.txt" % modifica guida adattiva, beccato
name = "logging_script__20260529_162703.txt" % cambio parametri e non ha funzionato
name = "logging_script__20260529_163128.txt" % velocità reimpostate e parametri cambiati, beccato
name = "logging_script__20260529_163842.txt" % guida adattiva 0.5 vs 0.25 target loro missato
name = "logging_script__20260529_164128.txt" % guida adattiva 0.4 vs 0.25 target loro missato
name = "logging_script__20260529_164518.txt" % guida base 0.4 vs 0.25 target loro preso dopo una vita (0.35 volte)
name = "logging_script__20260529_164923.txt" % aumentatà velocità poi come prima meno di una vita (0.25 volte)
name = "logging_script__20260604_104308.txt" % Target acclerante classico
name = "logging_script__20260604_104616.txt" % Sempre target accelerante
name = "logging_script__20260604_105643.txt" % Target accelerante super classico
name = "logging_script__20260604_110238.txt" % Target evasivo loro (0°)
name = "logging_script__20260604_110636.txt" % Target evasivo loro (45°)
name = "logging_script__20260604_111019.txt" % Target evasivo loro (45°) un po piu sodo
name = "logging_script__20260604_111323.txt" % Target evasivo loro (45°) N aumentato
name = "logging_script__20260604_111622.txt" % Target evasivo (45°) N regolato
% name = "logging_script__20260604_125929.txt"
name = "logging_script__20260604_130815.txt" % apng target accelerante classico miss dist 0.04 pen 32.42
name = "logging_script__20260604_131219.txt" % pursuit con bias target accelerante classico miss dist 0.014 pen 23.75
name = "logging_script__20260604_131501.txt" % png con bias target accelerante classico miss dist 0.002 pen 14.6060
name = "logging_script__20260604_132156.txt" % hybrid drive target accelerante classico miss dist 0.0250 pen 26.2520
name = "logging_script__20260604_133107.txt" % hybrid drive target accelerante classico mis distance 0.008 pen 18.6230
name = "logging_script__20260604_135154.txt" % hybrid drive targer intelligente miss distance 0.0150 pen 34.03090
name = "logging_script__20260604_135459.txt" % hybrid drive target intelligente triangolo miss distance 0.038 total pen 39.38
name = "logging_script__20260604_135857.txt" % png target intelligente triangolo miss distance 0.024 total pen 22.3790
name = "logging_script__20260604_140134.txt" % pursuit target intelligente triangolo miss distance 0.041 total pen 45.8910
name = "logging_script__20260604_140733.txt" % hybrid_drive target intelligente triangolo miss distance 0.0270 total pen 34.3590
name = "logging_script__20260604_141112.txt" % hybrid drive target intelligente triangolo miss distance 0.0490 total pen 36.0220
name = "logging_script__20260604_142049.txt" % hybrid drive target intelligente triangolo miss distance 0.0040 total pen 29.9530
%%        
%%  *****************************************************        
%%  *                                                   *
%%  *              DATA PROCESSING PART                 *
%%  *                                                   *
%%  *****************************************************        
% 
%         
        
current_file = mfilename('fullpath');
[path, ~, ~] = fileparts(current_file);

%this is a patch! do not know why but the command reports a wrong path 
path = pwd;


core = fullfile(path, '../core_data/', name);
guidance = fullfile(path, '../guidance_data/', name);
kalman = fullfile(path, '../kalman_data/', name);
kalman_fix = fullfile(path, '../kalman_fix_data/', name);
private = fullfile(path, '../private_guidance_data/', name);
delimiterIn = ' ';
headerlinesIn = 1;
raw_core_data = importdata(core,delimiterIn,headerlinesIn);
raw_guidance_data = importdata(guidance,delimiterIn,headerlinesIn);
raw_kalman_data = importdata(kalman,delimiterIn,headerlinesIn); % valido da logging_script__20230111_173929 in poi
raw_kalman_fix_data = importdata(kalman_fix,delimiterIn,headerlinesIn); % valido da logging_script__20230111_173929 in poi
raw_private_data = importdata(private,delimiterIn,headerlinesIn); 

if isstruct(raw_core_data) core_data = raw_core_data.data;
else core_data = raw_core_data;
end

if isstruct(raw_guidance_data) guidance_data = raw_guidance_data.data;
else guidance_data = raw_guidance_data;
end

if isstruct(raw_kalman_data) kalman_data = raw_kalman_data.data;
else kalman_data = raw_kalman_data;
end

if isstruct(raw_kalman_fix_data) kalman_fix_data = raw_kalman_fix_data.data;
else kalman_fix_data = raw_kalman_fix_data;
end

if isstruct(raw_private_data) private_data = raw_private_data.data;
else private_data = raw_private_data;
end

clear core
clear raw_core
clear current_file delimiterIn headerlinesIn name path

%% Data extraction
% Extracted data
target_px = core_data(:,1);
target_py = core_data(:,2);
drone_px = core_data(:,3);
drone_py = core_data(:,4);
target_est_vx = core_data(:,5);
target_est_vy = core_data(:,6);
drone_est_vx = core_data(:,7);
drone_est_vy = core_data(:,8);
core_r = core_data(:,9);
core_sigma = core_data(:,10);
core_r_dot_ff = core_data(:,11);
core_sigma_dot_ff = core_data(:,12);
core_r_dot_kf = core_data(:,13);
core_sigma_dot_kf = core_data(:,14);
target_est_ax = core_data(:,16);
target_est_ay = core_data(:,17);
core_time = core_data(:,15);
drone_est_ax = core_data(:,18);
drone_est_ay = core_data(:,19);
core_r_kf = core_data(:,20);
core_sigma_kf = core_data(:,21);
core_yaw = core_data(:,22); %valido solo dal 9 gennaio 2023 15:54 in poi

guidance_r_dot = guidance_data(:,1);
guidance_sigma_dot = guidance_data(:,2);
guidance_time = guidance_data(:,3);
guidance_acc = guidance_data(:,4);
guidance_v2 = guidance_data(:,5);
guidance_omega = guidance_data(:,6);

% Old Kalman log
%{ 
kalman_x = kalman_data(:,1);
kalman_y = kalman_data(:,2);
kalman_z = kalman_data(:,3);
kalman_yaw = kalman_data(:,4);
kalman_vx_b = kalman_data(:,5);
kalman_vy_b = kalman_data(:,6);
kalman_time = kalman_data(:,7);
%}


kalman_ax_b = kalman_data(:,1);
kalman_ay_b = kalman_data(:,2);
kalman_az_b = kalman_data(:,3);
kalman_time = kalman_data(:,4);

kalman_fix_x = kalman_fix_data(:,1);
kalman_fix_y = kalman_fix_data(:,2);
kalman_fix_z = kalman_fix_data(:,3);
kalman_fix_yaw = kalman_fix_data(:,4);
kalman_fix_time = kalman_fix_data(:,5);

clear vicon_data internal_data

%%eliminate wrong drone positions
idx_dronepx_wrong = find(drone_px==0);
idx_dronepy_wrong = find(drone_py==0);
idx_dronepxy_wrong = intersect(idx_dronepx_wrong,idx_dronepy_wrong);
drone_px(idx_dronepxy_wrong) =NaN;
drone_py(idx_dronepxy_wrong) =NaN;




%% compute kinematic quantities
vicon_vx = diff(drone_px)./diff(core_time);
vicon_vy = diff(drone_py)./diff(core_time);
vicon_ax = diff(vicon_vx)./diff(core_time(2:end));
vicon_ay = diff(vicon_vy)./diff(core_time(2:end));
%compute acc and vel in body frame
vicon_vx_b = vicon_vx;
vicon_vy_b = vicon_vy;
vicon_ax_b = vicon_ax;
vicon_ay_b = vicon_ay;
drone_est_ax_b = vicon_ax;
drone_est_ay_b = vicon_ay;
for i=1:length(vicon_vx)
    yaw = core_yaw(i+1);
    R = [cos(yaw), sin(yaw); -sin(yaw), cos(yaw)];
    vel_body = R*[vicon_vx(i); vicon_vy(i)];
    vicon_vx_b(i) = vel_body(1);
    vicon_vy_b(i) = vel_body(2);
end
for i=1:length(vicon_ax)
    yaw = core_yaw(i+2);
    R = [cos(yaw), sin(yaw); -sin(yaw), cos(yaw)];
    acc_body = R*[vicon_ax(i); vicon_ay(i)];
    vicon_ax_b(i) = acc_body(1);
    vicon_ay_b(i) = acc_body(2);
    acc_body_est = R*[drone_est_ax(i); drone_est_ay(i)];
    drone_est_ax_b(i) = acc_body_est(1);
    drone_est_ay_b(i) = acc_body_est(2);
end
core_omega = diff(core_yaw)./diff(core_time);
core_omega = [core_omega; core_omega(end,:)];
%smooth di alcuni segnali
core_omega_smooth = smooth(core_omega,20,'loess');
vicon_ay_b_smooth = smooth(vicon_ay_b,40,'loess');


%% Animation of the guidance
% This section creates 2D animation of the guidance using target and drone
% trajectories.
figure('name', "2D Guidance Visualization in Vicon Reference Frame")
hold on
grid on
xlabel("Vicon x axis [m]",'Interpreter', 'latex')
ylabel("Vicon y axis [m]",'Interpreter', 'latex')
xlim([-1.5, 1.2])
ylim([-1.0, 2.0])
axis equal 
h = animatedline('Color', [1, 0, 0], 'LineWidth', 3);
f = animatedline('Color', [0, 0, 1], 'LineWidth', 3);

a = tic;
for i = 1:length(core_time)
    if(i == 1)                    
        target = plot(target_px(i),target_py(i), ...
					    'o','Color',[0, 0, 0], 'MarkerSize', 8,...
					    'MarkerFaceColor',[0, 0, 1.0]);
        drone = plot(drone_px(i),drone_py(i), ...
					    'o','Color',[1.0, 0, 0], 'MarkerSize', 8,...
					    'MarkerFaceColor',[1.0, 0, 0]);                    
    else
        drone.XData = drone_px(i);
        drone.YData = drone_py(i);
        target.XData = target_px(i);
        target.YData = target_py(i);
    end
    addpoints(h, drone_px(i), drone_py(i));
    addpoints(f, target_px(i), target_py(i));
    drawnow limitrate 
    pause(0.01)
end
set(gcf, 'Color', 'w');


%% Animation of the guidance
% This section creates 2D animation of the guidance using target and drone
% trajectories.
figure('name', "2D Guidance Visualization in Vicon Reference Frame")
set(gcf,'Position',[100 100 100+1224 100+500]);
subplot(2,2,[1 3]);
hold on
grid on
xlabel("NAV x axis [m]",'Interpreter', 'latex')
ylabel("NAV y axis [m]",'Interpreter', 'latex')
xlim([-1.5, 1.2])
ylim([-1.0, 2.0])
axis equal 
h = animatedline('Color', [1, 0, 0], 'LineWidth', 3);
f = animatedline('Color', [0, 0, 1], 'LineWidth', 3);
if 1
    %set test range limits (TRL) [xmin xmax ymin ymax]
    %TRL = [-1 1 -1 0.5]; %primo box usato
    TRL = [-1.4 1.3 -1.4 1.7]; %box dal 13 gennaio in poi
    plot(TRL([1 2 2 1 1]),TRL([3 3 4 4 3]),'r:');
end

subplot(2,2,2);
sigmadot =  animatedline('Color', [1, 0, 0], 'LineWidth', 1);
grid on
xlabel("Time [s]",'Interpreter', 'latex')
ylabel("$\dot \sigma$ [deg/s]",'Interpreter', 'latex')

subplot(2,2,4);
omega_cmd =  animatedline('Color', [0, 0, 1], 'LineWidth', 1);
omega =  animatedline('Color', [0, 1, 0], 'LineWidth', 1);
grid on
xlabel("Time [s]",'Interpreter', 'latex')
ylabel("$\omega$ [deg/s]",'Interpreter', 'latex')


a = tic;
for i = 1:length(core_time)
    subplot(2,2,[1 3]);
    if(i == 1)                    
        target = plot(target_px(i),target_py(i), ...
					    'o','Color',[0, 0, 0], 'MarkerSize', 8,...
					    'MarkerFaceColor',[0, 0, 1.0]);
        drone = plot(drone_px(i),drone_py(i), ...
					    'o','Color',[1.0, 0, 0], 'MarkerSize', 8,...
					    'MarkerFaceColor',[1.0, 0, 0]);                    
    else
        drone.XData = drone_px(i);
        drone.YData = drone_py(i);
        target.XData = target_px(i);
        target.YData = target_py(i);
    end
    addpoints(h, drone_px(i), drone_py(i));
    addpoints(f, target_px(i), target_py(i));
    
    %sigma dot plot
    subplot(2,2,2);
    addpoints(sigmadot, core_time(i), core_sigma_dot_ff(i));

    %actual and commande omega plot
    subplot(2,2,4);
    addpoints(omega, core_time(i), -core_omega_smooth(i)*180/pi);
    %find corresponding sample in guidance time
    [discard, idx_min] = min(abs(guidance_time-core_time(i)));
    addpoints(omega_cmd, guidance_time(idx_min), guidance_omega(idx_min));
    
    drawnow limitrate 
    pause(0.01)
end
set(gcf, 'Color', 'w');

%% Additional plots

figure (4);
plot(core_time(:,1),core_r(:,1))    
hold off;
grid on;
xlabel("Time [s]",'Interpreter', 'latex');
title("Relative distance")
axis tight

%% Maybe it could be useful to plot the norm of the commanded acceleration
figure(5)
hold on 
grid on
xlabel("time")
title("Commanded acceleration")
plot(guidance_time, guidance_acc, 'r')

%% private data plot EXAMPLE
% 
figure(6);
hold on 
grid on
xlabel("time")
title("Sigma_R")
plot(private_data(:,1), private_data(:,4), 'r')


figure(7)
hold on 
grid on
xlabel("time")
title("fading filter comparison")
plot(core_time, core_sigma_dot_ff, 'r')
plot(guidance_time, private_data(:,6),'b')

%% Guidance challange 2024: Objective: Define metrics to evaluate different trials

% Score normalizations
max_consumption = 0.5;          % Maximum commanded acceleration
dist_threshold = 0.25;           % Below interception occurred
T_max = 40;                     % Maximum time allowed to run the scenario 


% Find where is the minimum distance along the data (hypothetical
% miss distance)

t_md = guidance_time(end);
md = Inf;
for i = 1:length(core_time)
    
    dist = core_r(i);  
    if dist < md
        md = dist;
        md_index  = i;
        t_md = core_time(md_index);
    end

end

INTERCEPTION = md < dist_threshold; % Did the interception actually occur?

% take time and commanded acceleration only up to miss_distance point
t_init = guidance_time(1);
t_end = guidance_time(end);

% Give maximum penalty in case scenario did not end with intercept
if INTERCEPTION
    t_span = t_md - t_init;
    consumption = (1/t_span) * trapz(guidance_time, abs(guidance_acc));
else
    md = dist_threshold;
    t_span = T_max - t_init;
    consumption = max_consumption;
end

% Then design Weights K1 K2 K3
K1 = 55;
K2 = 25;
K3 = 20;

P = K1 * (md/dist_threshold) + K2 * (consumption/max_consumption) + K3 * t_span/(T_max - t_init); 

fprintf("##################### SCORES ########################\n")
fprintf("-------------- Miss Distance ---------------\n")
disp(round(md, 3))
if INTERCEPTION
    fprintf("-------------- Consumption ----------------\n")
    disp(round(consumption, 3))
    fprintf("----------- Time of interception-----------\n")
    disp(round((t_md - t_init), 3))
else
    fprintf("-------------- Consumption ----------------\n")
    disp(round(max_consumption, 3))
    fprintf("----------- Time of interception-----------\n")
    disp(round((T_max - t_init), 3))
end

fprintf("----------- Total Penalization ------------\n")
disp(round(P, 3))


