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

load('OUT.mat');
close all

% Plot Position
figure();
plot(tout, OUT.EOM.X_E.Data(:,1));
hold on;
plot(tout, OUT.EOM.X_E.Data(:,2));
hold on;
plot(tout, -1.*OUT.EOM.X_E.Data(:,3));

legend('X_E', 'Y_E', 'Z_E');

% Plot Velocities
figure();
plot(tout, OUT.EOM.V_E.Data(:,1));
hold on;
plot(tout, OUT.EOM.V_E.Data(:,2));
hold on;
plot(tout, OUT.EOM.V_E.Data(:,3));

legend('VE_x', 'VE_y', 'VE_z');

% Plot Velocities - WInd Axis
figure();
plot(tout, OUT.EOM.V_W.Data(:,1));
hold on;
plot(tout, OUT.EOM.V_W.Data(:,2));
hold on;
plot(tout, OUT.EOM.V_W.Data(:,3), '+');

legend('VW_x', 'VW_y', 'VW_z');

% Plot Angles of Attack
figure();
plot(tout, OUT.EOM.STAB_ANGLES.Data(:,1));
hold on;
plot(tout, OUT.EOM.STAB_ANGLES.Data(:,2));
legend('\alpha', '\beta');

% Plot Masses
figure();
plot(tout, OUT.MASS.MASS_VEC.Data(:,1));
hold on;
plot(tout, OUT.MASS.MASS_VEC.Data(:,2));
hold on;
plot(tout, OUT.MASS.MASS_VEC.Data(:,3));
legend('m_{x,w}', 'm_{y,w}', 'm_{z,w}');
title('Mass Time History');

% Plots Thrust
figure();
plot(tout, OUT.MOT.THR_VEC.Data(:,1));
hold on;
plot(tout, OUT.MOT.THR_VEC.Data(:,2));
hold on;
plot(tout, OUT.MOT.THR_VEC.Data(:,3));
title('Thrust Profile');
legend('T_{x,w}', 'T_{y,w}', 'T_{z,w}');

% Plot Thrust Vectors
figure();
plot(OUT.MOT.THR_MOM_VEC.Time, OUT.MOT.THR_MOM_VEC.Data(:,1));
hold on;
plot(OUT.MOT.THR_MOM_VEC.Time, OUT.MOT.THR_MOM_VEC.Data(:,2));
hold on;
plot(OUT.MOT.THR_MOM_VEC.Time, OUT.MOT.THR_MOM_VEC.Data(:,3));
legend('T_{x,w}', 'T_{y,w}', 'T_{z,w}');
title('Thrust Moment');