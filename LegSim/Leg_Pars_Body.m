function body = Leg_Pars_Body
% Parameters of the bodies of the multi-rigid-body dynamic model
% for the leg simulation, comprising both human (residual side) and prosthesis

%% Joint Transformations (lenghts and angles)

% default above-knee (patient) dimensions:
body.l_trunk = 0.6; % length: neck - hip joint
body.l_thigh = 0.42; % length: hip joint - knee joint

% default prosthesis dimensions
body.l_shank = 0.42; % length: knee joint - ankle joint (consists of calf, pylon and ankle mount)
body.l_sens = 0.28; % length: knee joint - tube sensor

% default ankle and foot dimensions:
body.y_foot = 0.08; % y-coordinate: ground -> ankle joint
body.x_heel = -0.025; % x-coordinate: ankle joint -> foot heel
body.x_ball = 0.150; % x-coordinate: ankle joint -> foot ball

% q0 joint transformations
body.p0_ankle = 4/180*pi; % ankle physical offset angle

%% Masses (trunk and thigh are patient mass)

body.m_trunk = 62.5; % includes other leg, head, arms
body.m_thigh = 9; % human thigh (incl. socket, excl. prosthesis)
body.m_shank = 2.8; % prosthesis (incl. pyramid & pylon)
body.m_foot = 0.7; % foot (incl. ankle & pyramid)

%% Center of mass & inertia

% trunk (+head +arms +other leg) (estimated)
body.c_trunk = [0;body.l_trunk/2]; %
body.I_trunk = body.m_trunk*body.l_trunk^2/12*2; % 2x slender beam

% thigh (approximated as a wedge, ratio 3)
[body.I_thigh, c_thigh_y] = MoI_2D_line({'wedge'},...
    {[0 -body.l_thigh 4]},...
    body.m_thigh);
body.c_thigh = [0;c_thigh_y];

% shank = calf & pylon
body.c_shank = [-0.01;-0.12];
body.I_shank = 0.03;

% foot inertia (estimated)
body.c_foot = [body.x_heel/2+body.x_ball/2;-body.y_foot*2/3];
body.I_foot = body.m_foot*(body.x_ball-body.x_heel)^2/12; % slender beam

% also save the gravity here
body.g = 9.81; % [m/s^2];