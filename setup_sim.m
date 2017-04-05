clear;
clc;
close all;

%% Add paths for modules

load_folders;

%% Load Module Parameters

MOT = setup_mot('create');
MOT = setup_mot('update', MOT);

%% Set Simulation Parameters

test_model;
set_param(bdroot, 'StopTime', '500');

%% Start Sim

% set_param(bdroot, 'SimulationCommand', 'start');
sim('test_model');

%% Post-Process Results

OUT = load('OUT.mat');

% Plot Position
figure();
plot(tout, OUT.ans.EOM.X_E.Data(:,1));
hold on;
plot(tout, OUT.ans.EOM.X_E.Data(:,2));
hold on;
plot(tout, OUT.ans.EOM.X_E.Data(:,3));

legend('X_E', 'Y_E', 'Z_E');

% Plot Velocities
figure();
plot(tout, OUT.ans.EOM.V_E.Data(:,1));
hold on;
plot(tout, OUT.ans.EOM.V_E.Data(:,2));
hold on;
plot(tout, OUT.ans.EOM.V_E.Data(:,3));

legend('VE_x', 'VE_y', 'VE_z');

% Plot Masses
figure();
plot(OUT.ans.MASS.MASS.Time, OUT.ans.MASS.MASS.Data);

% Plots Thrust
figure();
plot(OUT.ans.MOT.Thrust.Time, OUT.ans.MOT.Thrust.Data);
