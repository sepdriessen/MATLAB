function ctrl = Leg_Pars_Ctrl_VariableDamping
% Prosthesis controller parameters of a variable damping leg.
% Sub-divided in structures:
%
% pre: pre-processing of data inputs (signal processing/filtering)
% fsm: finite-state-machine (high-level ctrl)
% mid: mid-level control

ctrl.t_step = 1/200; % [s] (discrete) time step, sampling time, used by controllers

% preprocessing: signal processing, filtering:
pre.freq_fast = 15; % cut-off frequency fast filter
pre.freq_normal = 5; % cut-off frequency normal filter

% FINITE STATE MACHINE (FSM) PARAMETERS
% temporal parameters
fsm.t_0 = 0.05; % minimum general duration (sometimes conditional) of a state
fsm.t_3 = 0.10; % conditional timer for PSw
fsm.t_stX = 0.6; % time to (release constraints to) switch back to stance (StF)

% force parameters
fsm.F_St = 0.05; % (inc) force switch for going to stance
fsm.F_Sw = 0.05; % (dec) force switch for going to swing

% angular velocity parameters
fsm.w_0 = 0/30*pi; % (dec-/inc+) general switching velocity (+-), used for >StE and >SwE
fsm.w_1 = 15/30*pi; % (inc) flexing velocity for StE->StF 
fsm.w_stR = 5/30*pi; % (dec) shuffling velocity SwF>StF

% angular position parameters
fsm.p_st = 50/180*pi; % (dec!) switch to StF (from SwE), leg sufficiently extended
fsm.p_stR = 20/180*pi; % (dec!) recovery switch to StF (from SwF)
fsm.p_2 = 15/180*pi; % (dec) extension to StE 
fsm.p_3 = 15/180*pi; % (dec) (more) extension to PSw
fsm.p_4 = 15/180*pi; % (inc) ->SwF
fsm.p_5 = 90/180*pi; % (dec) ->SwE

% moment parameters
fsm.M_3 = 0.5; % (inc) moment switch ->PSw
fsm.M_stX = 0.1; % (dec) recovery to Stance from PSw.

% basic damping:
mid.c_StF = 20;
mid.c_StE = 20;
mid.c_PSw = 0;
mid.c_SwF = 0;
mid.c_SwE = 0;

% end stop angles and dampings 
mid.p_SwFMax = 60/180*pi;
mid.p_SwEMax = 15/180*pi;
mid.c_SwFMax = 50;
mid.c_SwEMax = 200;

%% concatenate outputs
ctrl.pre = pre;
ctrl.mid = mid;
ctrl.fsm = fsm;
