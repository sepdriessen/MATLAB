function device = Leg_Pars_Device_VariableDamping
% Prosthesis device parameters of a variable damping leg.

%% knee prosthesis parameters

% joint damper parameters
knee.damper.T_stic = 0.1; % [Nm] basic stiction (applied using 100*htan(speed))
knee.damper.c = 0.2; % [Nms/rad] linear basic damping coefficient
knee.damper.k = 2; % [Nm/rad] linear (rotational) spring stiffness

% extension endstop
knee.ext_endstop.p_0 = 3/180*pi; % starting (touching) knee angle for ext
knee.ext_endstop.T_half = 100; % torque at half thickness compression
knee.ext_endstop.p_tot = 2/180*pi; % thickness end stop (2 degree)
knee.ext_endstop.t_c = 0.01; % recovery time constant

% flexion endstop
knee.flx_endstop.p_0 = 115/180*pi; % starting (touching) knee angle for flex
knee.flx_endstop.T_half = 100;
knee.flx_endstop.p_tot = 2/180*pi;
knee.flx_endstop.t_c = 0.01;

% encoder(s) properties
knee.encoder.res = 2^12; %
knee.encoder.offset = 0; % offset in ticks

%% tube sensor parameters
% no quantization applied right now, so these take no effect.
tube.offset_F = 0; % force sensor zero value [ticks]
tube.gain_F = 0.1; % force sensor multiplier [ticks/N]
tube.offset_M = 0; % moment sensor zero value [ticks]
tube.gain_M = -10; % moment sensor multiplier [ticks/N/m]

%% ankle prosthesis parameters

% passive spring-damper system
ankle.k = 6*75; % 0.1*75*180/pi (ca. 0.1 Nm/kg/deg) CHANGED
ankle.c = 1;

%% concatenate outputs
device.knee = knee; % knee prosthesis data
device.tube = tube; % tube data
device.ankle = ankle; % ankle prosthesis data