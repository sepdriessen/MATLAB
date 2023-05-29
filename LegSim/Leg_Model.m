function robot = Leg_Model(body)
% function robot = Leg_Model(BODY)
% create robot structure readable by spatial_v2 software from body
% parameter structure BODY that describes a human leg with a knee and
% an ankle prosthesis in the sagittal plane.
%
% These are the 8 bodies of the model:
% body #: body name [joint name] (note)
% 1: -- [trunk x] (massless)
% 2: -- [trunk y] (massless)
% 3: trunk [trunk rot] (incl. arms, intact leg, head)
% 4: upper leg [hip]
% 5: lower leg [knee]
% 6: -- [tube y] (massless)
% 7: prosthesis tube [tube rot] (massless)
% 8: prosthesis foot [ankle]

%% define properties

% rigid-body inertia matrices
I_trunk = mcI( body.m_trunk, body.c_trunk, body.I_trunk ); % body 3
I_thigh = mcI( body.m_thigh, body.c_thigh, body.I_thigh); % body 4
I_shank = mcI( body.m_shank, body.c_shank, body.I_shank ); % body 5
I_foot = mcI( body.m_foot, body.c_foot, body.I_foot ); % body 8

%% define the robot

robot.NB = 8;
robot.parent = [0 1 2 3 4 5 6 7];
robot.jtype = { 'px', 'py', 'r', 'r', 'r', 'py', 'r', 'r'};
l_pylon = body.l_shank - body.l_sens; % pylon length
robot.Xtree = { eye(3), eye(3), eye(3), eye(3),...
    plnr( 0, [0,-body.l_thigh]),... % hip -> knee
    plnr( 0, [0,-body.l_sens]),eye(3),... % knee -> tube sensor 
    plnr( body.p0_ankle, [0,-l_pylon])}; % tube sensor -> foot
robot.I = { zeros(3), zeros(3), I_trunk, I_thigh, ...
    I_shank, zeros(3), zeros(3), I_foot};
robot.gravity = [0;-body.g];

%% ground contact

% define contact points
robot.gc.point = [0 0 0 0 body.x_heel body.x_ball;
    body.l_trunk 0 0 0 -body.y_foot*[1 1]];
robot.gc.body = [3 4 5 8 8 8];

% add ground contact parameters
robot.gc.K = 2e5; % ground stiffness
robot.gc.D = 2e4; % ground damping
robot.gc.mu = 0.5; % ground friction coefficient

%% appearance

% appearance-related parameters (local)
rbeam=0.02; % beam radius (for cylindrically-shaped bodies)
lfwidth=0.025; % foot width (transversal)
ltwidth=0.075; % trunk width (planar)
rjoint=0.03; % joint radius
mycol=lines(7); % color map used

% define appearance
robot.appearance.base = ...
    {'tiles', [-0.5 0.5; 0 0; -0.5 0.5], 0.1, ...
     'colour', [1 0 0], 'line', [0 0 0; 0.5 0 0], ...
     'colour', [1 0 0], 'line', [0 0 0; 0 0.5 0]
      };
robot.appearance.body{3} = ...
    {'colour', mycol(3,:),... 
     'cyl', [0 0 0; 0 0 -rjoint], ltwidth, ...
     'cyl', [0 body.l_trunk 0; 0 body.l_trunk -rjoint], ltwidth, ...
     'box', [-ltwidth 0 0;ltwidth body.l_trunk -rjoint]
    };
robot.appearance.body{4} = ...
    {'colour', mycol(5,:),... 
     'cyl', [0 0 0; 0 -body.l_thigh 0], rbeam, ...
     'cyl', [0 0 0; 0 0 rjoint], rjoint, ...
     'cyl', [0 -body.l_thigh 0; 0 -body.l_thigh -rjoint], rjoint
    };
robot.appearance.body{5} = ...
    {'colour', mycol(1,:),... 
     'cyl', [0 0 0; 0 -body.l_sens 0], rbeam, ...
     'cyl', [0 0 0; 0 0 rjoint], rjoint
    };
robot.appearance.body{7} = ...
    {'colour', mycol(6,:),... 
     'cyl', [0 0 0; 0 -l_pylon 0], rbeam, ...
     'cyl', [0 -l_pylon 0; 0 -l_pylon -rjoint], rjoint
    };
robot.appearance.body{8} = ...
    {'colour', mycol(2,:),... 
     'cyl', [0 0 0; 0 -body.y_foot 0], rbeam, ...
     'cyl', [0 0 0; 0 0 rjoint], rjoint, ...
     'vertices',[0 0 lfwidth;
                 body.x_heel -body.y_foot lfwidth;
                 body.x_ball -body.y_foot lfwidth;
                 0 0 -lfwidth;
                 body.x_heel -body.y_foot -lfwidth;
                 body.x_ball -body.y_foot -lfwidth], ...
     'triangles', [1 2 3;
                   4 6 5;
                   1 5 2;
                   1 4 5;
                   1 3 6;
                   1 6 4;
                   2 6 3;
                   2 5 6]
    };

%% camera settings
robot.camera.body = 3; % track trunk
robot.camera.up = [-robot.gravity' 0]; % up is the direction opposing gravity
robot.camera.trackpoint = [0 0 0];
robot.camera.locus=[0 0];