function device = Leg_Pars_Device_Powered
% Prosthesis device parameters of a powered leg.

%% knee prosthesis parameters

% actuator parameters (motor + gearbox)
knee.motor.Ktau = 36.9e-3; % [Nm/A] motor torque constant
knee.motor.Rwind = 0.608; % [ohms] motor winding resistance
knee.motor.Irot = 0.181e-4+0.039e-4; % [kg m^2] fast shaft inertia (rotor+shaft)
knee.motor.Imax = 6.4; %[A] max current
knee.motor.Umax = 24; % [V] maximum supply voltage
knee.motor.Ngear = 100; % gearbox ratio
knee.motor.etamax = 0.8; % maximum efficiency
knee.motor.Tstiction = 0.023; % [Nm] starting torque
knee.motor.Tviscous = [0.035 0.035]; % [Nm] running torque
knee.motor.wviscous = [500 10000]/30*pi; % [rad/s] referencodere speed for running torque

% extension endstop
knee.ext_endstop.p_0 = 3/180*pi; % starting (touching) knee angle for ext
knee.ext_endstop.p_tot = 2/180*pi; % thickness end stop (2 degrees)
knee.ext_endstop.T_half = 100; % torque at half thickness compression
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
ankle.k = 6*75; % 0.1*75*180/pi (ca. 0.1 Nm/kg/deg)
ankle.c = 1;

%% concatenate outputs
device.knee = knee; % knee prosthesis data
device.tube = tube; % tube data
device.ankle = ankle; % ankle prosthesis data

