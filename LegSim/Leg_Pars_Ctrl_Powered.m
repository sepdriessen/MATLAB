function ctrl = Leg_Pars_Ctrl_Powered
% Prosthesis controller parameters of a powered leg.
% Sub-divided in structures:
%
% pre: pre-processing of data inputs (signal processing/filtering)
% mid: mid-level control

ctrl.t_step = 1/200; % [s] (discrete) time step, sampling time, used by controllers

% preprocessing: signal processing, filtering:
pre.freq_fast = 15; % cut-off frequency fast filter
pre.freq_normal = 5; % cut-off frequency normal filter

% FINITE STATE MACHINE (FSM) PARAMETERS
% internally defined

% MID-LAYER PARAMETERS
% extension current for overflexion
mid.Imax = 6.4; % max and extension current for knee angles>p_flxlim
% thresholds
mid.p_flxlim = 55/180*pi;
% target angles
mid.p_hs = 4/180*pi;
mid.p_swing = 65/180*pi;
mid.p_ext = 15/180*pi;
% stiffness gains
mid.k_1 = 0.150/pi*180;
mid.k_2 = 0.500/pi*180;
mid.k_4 = 0.02/pi*180;
mid.k_5 = 0.02/pi*180;
% damping gains in s/rpm (directly applied to motor velocity = 100x joint)
mid.c_1 = 0;
mid.c_2 = 0; 
mid.c_3 = 0;
mid.c_5 = 0;
mid.c_end = 0.01;


%% concatenate outputs
ctrl.pre = pre;
ctrl.mid = mid;
