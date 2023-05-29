function human = Leg_Pars_Human(body)
% Human controller parameters

%% rough reference (ref) trajectories, normalized by time
% trunk rotation trajectories
human.ref.pt_stand = 0/180*pi;
human.ref.pt_walk = -4/180*pi;
% hip and thigh rotation trajectories
human.ref.ph_stand = 0/180*pi;
ph_max = 25/180*pi;
ph_minstart = -15/180*pi;
ph_min = -10/180*pi;
human.ref.th_walk = [0;0.25;0.5;0.51;1];
human.ref.ph_walk = [ph_max;ph_min;ph_min;ph_max;ph_max];
human.ref.th_walkstart = [0;0.25;0.5;0.51;1];
human.ref.ph_walkstart = [0;ph_minstart;ph_minstart;ph_max;ph_max];
human.ref.th_walkstop = [0;0.25;0.5;0.51;0.75];
human.ref.ph_walkstop = [ph_max;ph_min;ph_min;ph_max;0];

human.ref.ph_max = ph_max; % store, because used for initialization

%% smooth reference trajectory properties 
% PD position filter of rough refs using the following damping ratio:
human.ref.Z_DR = sqrt(0.5);

%% impedance properties 
% PD torque control of smooth references, for FD.

% standing mode:
% trunk torque
human.imp.Pt_stand = 500;
human.imp.Dt_stand = 100;
% hip torque ctrl
human.imp.Ph_stand = 500;
human.imp.Dh_stand = 100;

% walking mode:
% trunk torque
human.imp.Pt_walk = 5000;
human.imp.Dt_walk = 500;
% hip torque ctrl
human.imp.Ph_walk = 2000;
human.imp.Dh_walk = 100;

%% SLIP properties

% length
l_leg = body.l_thigh + body.l_shank + body.y_foot;
human.SLIP.l_rest = l_leg; % [m] SLIP rest length

% attack angle
human.SLIP.pk_gc = 6/180*pi; % estimated equivalent knee angle at ground contact

%% initial controller parameters (updated by controller)
% estimated step length and time based on SLIP
human.x_step_0 = 4/3; % [m]
% healthy leg thrust force gain (for energy injection)
human.Kthrust_0 = 50; % initial value