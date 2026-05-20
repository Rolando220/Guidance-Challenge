%% SGN_run_adapted_truePNG.m
% Minimal run script for the adapted model.
%
% 1) Put this folder on the MATLAB path.
% 2) Open/copy truePNG.mdl as truePNG_drone_adapted.mdl.
% 3) Apply the Simulink modifications described in README_MODIFICHE_SIMULINK.txt.
% 4) Run this script.

clear; clc; close all

thisFolder = fileparts(mfilename('fullpath'));
addpath(thisFolder);

SGN_drone_challenge_ini;

modelName = 'SGN_guidance_simulator';

if exist([modelName '.slx'], 'file') || exist([modelName '.mdl'], 'file')
    load_system(modelName);
    set_param(modelName, 'StopTime', num2str(STOP_TIME));
    simOut = sim(modelName);
else
    warning('Model %s not found', modelName);
end
%prova

results;
