% Script to run simulink model of a human wearing a variable damping
% prosthetic knee in combination with a passive ankle.
% Includes socket dynamics.
%
% More precisely, this script:
% - defines the human intent
% - loads parameters and creates the system
% - initalizes the system
% - runs the simulation
% - plots some results
% - shows an animation of the dynamic model (and possibly saves it)

clear

%% main inputs
% human
t_intent = [0;1;35];
intent = [0;1;0];  % 0 = standing still, 1 = walking
t_intentv = [0; 9.99; 10; 14.99; 15; 20; 25;35];
intentv = [1.1; 1.1; 1.4; 1.4; 1.1; 1.1; 1.4;0.8];

% simulation
t_end = 42; % stop time [s]

%% load parameters and generate model

% human-prosthesis system
body = Leg_Pars_Body; % default multibody (human+prosthesis) parameters
device = Leg_Pars_Device_VariableDamping; % prosthesis-specific device parameters
human = Leg_Pars_Human(body); % default human controller parameters
robot = Leg_Model_Socket(body); % dynamic model definition (spatialv2)

% save mass in body structure
ret = EnerMo(robot,zeros(robot.NB,1),zeros(robot.NB,1)); % obtain mass
body.m = ret.mass; 
clear ret;

% add socket device parameters
device.socket.k_ext = body.m*body.g/0.01;
device.socket.c_ext = 2*sqrt(device.socket.k_ext*body.m);
device.socket.k_rot = 0.5*body.m/0.01;
device.socket.c_rot = 2*sqrt(device.socket.k_rot*body.m/4);

% prosthesis controller
ctrl = Leg_Pars_Ctrl_VariableDamping;

% choose forward or inverse dynamics joints
FD = [1;1;0;1;1;1;1;0;0;1]; % forward dynamic array (1=fd,0=id)

% joiht numbers:
j_trunkx = 1;
j_trunky = 2;
j_trunk_rot = 3;
j_hip = 4;
j_socket_ext = 5;
j_socket_rot = 6;
j_knee = 7;
j_tube_ext = 8;
j_tube_rot = 9;
j_ankle = 10;

%% initialization procedures

% calc human SLIP angle of attack
xgc = gcPosVel(robot,[0 0 0 human.ref.ph_max 0 0 -human.SLIP.pk_gc 0 0 0],...
    zeros(robot.NB,1));
y_pen_0 = -xgc(2,5);%+75*9.81*(1/sys.HL.k_pen-1/robot.gc.K);
human.SLIP.p_attack = acos(y_pen_0/human.SLIP.l_rest);

% save mass in body structure
ret = EnerMo(robot,zeros(robot.NB,1),zeros(robot.NB,1)); % obtain mass
body.m = ret.mass; 
clear ret;

% Initial conditions dynamic model
% depend on initial intent (standing or walk)
ref = human.ref; % shorten
if intent(1)==0 % standing still
    q0 = [0;1;ref.pt_stand;
        0;0; % socket
        ref.ph_stand-ref.pt_stand;-device.knee.ext_endstop.p_0;0;0;0]; % init pos
    qd0 = [0;0;0;0;0;0;0;0;0;0]; % init vel
else % walk cycle
    q0 = [0;1;ref.pt_walk;
        0;0; % socket
        ref.ph_walk(1)-ref.pt_walk;-human.SLIP.pk_gc;0;0;0]; 
    qd0 = [intentv(1)*1.25*intent(1);0;0;0;0;0;0;0;0;0]; % start 25% higher <- CHECK
end

u0 = zeros(1,length(robot.gc.body)); % ground contact init deformations

% update initial torso height to make proper ground contact if FD
if FD(2)
    gc = gcPosVel(robot,q0,qd0);
    q0(2) = q0(2) - min(gc(2,:)) + body.m*robot.gravity(2)/robot.gc.K;
end
clear ref

% other preparations
robsim = robotcell2mat(robot); % remove cells for use in Simulink

%% run simulation
% simulation settings
simOpts.StartTime ='0'; % [s]
simOpts.StopTime = num2str(t_end);  % [s]

% choose a solver
% var step solvers:
%   ode45, ode23, ode113, ode15s, ode23s, ode23t, ode23tb
% can deal with (moderately) stiff ground contact events:
%   ode15s, ode23s, (ode23t,) ode23tb
% can deal with state dependency:
%   ode15s, ode23t, ode23tb
% can deal with singular mass and initial slopes:
%   ode15s, ode23t
simOpts.Solver ='ode23s'; % ode15s is too slow
% solver parameters
simOpts.AbsTol = '1e-3';
simOpts.RelTol = '1e-2';
simOpts.MaxStep = num2str(ctrl.t_step); % maximum sample time step size [s]

% actually run the simulation
tic % start timer to measure simulation time
% out=sim('LegSim',simOpts);
% out=sim('LegSimSingle',simOpts);
out=sim('Leg_Sim_Socket_VariableDamping',simOpts);
% out=sim('LegSimDM',simOpts);
toc % stop timer (note: first time building takes more time)

%% plot section
% in order to easily find figures, use the following figure identifiers:

energyAudit = 3;

kneePos = 11;
kneeVel = 12;
kneeTorque = 13;

tubeForce = 15;
tubeMoment = 16;
tubeMomentD = 17;

hipPos = 21;
hipTorque = 22;

trunkHeight = 24; % (= vertical position hip joint)
trunkForces = 25; % (= forces on hip joint)

footHeight = 26;

ambulationSpeed = 29;

socketDisplacements = 31;
socketForces = 32;

%% energy audits
% Draw energy audit, which shows all energy levels to, from and within
% the system, which should in total remain constant. Types of energies:
% P: potential energy (gravity and springs) [retrievable].
% K: kinetic energy (motion) [retrievable].
% H: heat loss (friction), [irreversibly lost, only increases]
% W: work sources (other joints, actuators) [any, but typically decreases]

t = out.tout;

E_g = zeros(length(t),1);
cm = zeros(length(t),2);
vcm = zeros(length(t),2);
E_k = E_g;
E_kCoM = E_k;
for i1=1:length(out.tout)
    ret=EnerMo(robot,out.q(i1,:),out.qd(i1,:));
    cm(i1,:) = ret.cm';
    vcm(i1,:) = ret.vcm';
    E_g(i1) = -ret.PE;
    E_k(i1) = ret.KE;
    E_kCoM(i1) = 0.5*norm(ret.vcm)^2*ret.mass;
end
E_g = E_g-min(E_g);
E_kRest= E_k - E_kCoM;
E_trunk = -sum(out.E_joint(:,1:3),2);
E_trunk = E_trunk - min(E_trunk);
E_hip = -out.E_joint(:,4);
E_hip = E_hip - min(E_hip);
E_act = max(out.E_act) - out.E_act;

figure(energyAudit),clf
harea=area(out.tout,[...
    E_g ... % gravitational potential energy
    out.E_ps out.E_esFlx(:,2)+out.E_esExt(:,2) out.E_ankle(:,1)  ... % internal potential energy (spring)
    E_kCoM E_kRest ... % kinetic energy
    E_act ... % actuator (damper)
    -out.E_fric out.E_esFlx(:,4)+out.E_esExt(:,4) out.E_ankle(:,2) sum(out.E_socket,2) ... % internal wasted energy (friction)
    -out.E_gcStick -out.E_gcSlip... % external wasted energy (gc)
    E_hip E_trunk... % work done by the system
    ]); 
hold on
mycol=[[0.75 0.5 1]; % gravitational potential energy
       [1;0.75;0.5]*[0.5 0.5 1]; % internal potential energy (springs)
       [1;0.75]*[0.5 1 0.5]; % kinetic energy
       [0.95 0.9 0.5]; % knee actuator
       [1;0.75;0.5; 0.35]*[1 0.75 0.5]; % internal wasted energy (dampers)
       [1;0.75]*[1 0.5 0.5] % external wasted energy (gc)
       [0.75;0.5]*[1 1 1] % work done by ID
       ];
for i1=1:length(harea)
harea(i1).FaceColor = mycol(i1,:);
end
legend('gravity (P)',...
    'damper (P)','end stops (P)','ankle (P)',...
    'mass (K)','inertia (K)',...
    'knee actuator (W)',...
    'damper (H)','end stops (H)','ankle (H)','socket (H)',...
    'ground damp (H)','ground slip (H)',...
    'hip (W)','trunk (W)',...
    'location','northeastoutside')
xlabel('time [s]'),ylabel('energy [J]')
title('Energy audit (total = constant)')
set(gca,'Layer','top')

%% other plots
t_ctrl = out.t_ctrl;
t_step = ctrl.t_step;
pre = ctrl.pre;
fsm = ctrl.fsm;

plotstates=@()plot(t_ctrl,filly(out.state,get(gca,'ylim'),[0.8 1]),'m');
n=length(t_ctrl);

% knee position
figure(kneePos),clf
plot(t_ctrl,out.pk*180/pi);hold on
plot(t_ctrl,out.F*body.g);hold on
xlabel('time [s]');ylabel('angle [deg] / force density [N/kg]')
plotstates();
legend('knee angle [deg]',...
    'normalized tube force [N/kg]','states')
set(gca,'ylim',[-10,120]);
grid on

% knee velocity
figure(kneeVel),clf
% plot(t_ctrl,out.ikd_raw/sys.kne.enc.ik_rev*60);hold on
plot(t_ctrl,out.wk*30/pi);hold on
set(gca,'ylim',[-60 60])
plot(t_ctrl,out.wk_ffast*30/pi);
plotstates();
xlabel('time [s]');ylabel('knee velocity [rpm]')
legend('vel raw',... 'vel in',...
    ['vel fast (fc = ' num2str(pre.freq_fast) ' Hz)'],...
    'state')
grid on

% knee torque
figure(kneeTorque),clf
plot(t_ctrl,out.cmd_m);hold on
plot(t,out.T_act);
plot(t,out.T_fric);
plot(t,out.T_spring);
plot(t,out.T_esExt+out.T_esFlx);
plot(t,out.T_act+out.T_fric+out.T_spring+out.T_esExt+out.T_esFlx,'--');
plotstates();
xlabel('time [s]');ylabel('knee motor torque [Nm]')
legend('actuator cmd (quadratic damping)',' actuator torque',...
    'knee friction','knee spring','end stops','knee torque','state')
grid on

% force sensor
figure(tubeForce),clf
plot(t_ctrl,out.F);hold on
plot(t_ctrl([1 end]),[1 1]*fsm.F_St,'--');
plot(t_ctrl([1 end]),[1 1]*fsm.F_Sw,'--');
plotstates();
xlabel('time [s]');ylabel('force per unit of body weight [N/N]')
legend('force: raw','stance threshold','swing threshold','states')
grid on

% moment sensor
figure(tubeMoment),clf
plot(t_ctrl,out.M);hold on
xlabel('time [s]');ylabel('moment density [Nm/kg]')
plotstates();
grid on

% moment sensor
figure(tubeMoment),clf
plot(t_ctrl,out.M);hold on
xlabel('time [s]');ylabel('moment density [Nm/kg]')
plotstates();
grid on

% hip joint trajectory
figure(hipPos),clf
plot(t,out.qcmd(:,4)*180/pi,'-.'),hold on
plot(t,[out.qref(:,4)*180/pi out.qdref(:,4)*30/pi out.qddref(:,4)])
plot(t,out.q(:,4)*180/pi,'linewidth',2);
plotstates();
grid on
xlabel('time [s]'),ylabel('hip angle (+dervs) [deg, rpm, rad/s^2]')
legend('ref','pos','vel','acc','real pos','states')

% hip torque (tau4)
figure(hipTorque),clf
plot(t,out.tau(:,4)),hold on
plot(t,-out.tau(:,3))
plotstates();
xlabel('time [s]'),ylabel('hip torque [Nm]')
legend('prosthetic side','healthy side','states')
grid on

% trunk (hip joint) height (q2)
figure(trunkHeight),clf
plot(t,out.q(:,2)),hold on
SLIP = human.SLIP;
plot(t([1 end]),[1 1]*cos(SLIP.p_attack)*SLIP.l_rest)
plotstates();
xlabel('time [s]'),ylabel('hip height [m]')
grid on

% trunk forces = forces on hip joint (tau1,tau2)
figure(trunkForces),clf
plot(t,out.tau(:,1:2))
xlabel('time [s]'),ylabel('force on hip [N]'),legend('x','y')
grid on

% foot: heel and ball height
figure(footHeight),clf
plot(t,squeeze(out.xc(2,[5:end],:)));hold on
plotstates();
xlabel('time [s]'),ylabel('foot height [m]')
legend('heel','ball (toe)','states')
grid on

% horizontal velocity
figure(ambulationSpeed),clf
[B,A] = butter(1,0.5*2*t_step);
% q1d
plot(t,out.qd(:,1));hold on
plot(t_ctrl,filtfilt(B, A, interp1(t,out.qd(:,1),t_ctrl)),'linewidth',2);
% com
% plot(t,vcm(:,1));hold on
% plot(t_ctrl,filtfilt(B, A, interp1(t,vcm(:,1),t_ctrl)),'linewidth',2);
% rest
plot(t,out.intentv.*(out.intent==1),'--','linewidth',2);
plot(t,out.v_amb.*(out.intent==1),'linewidth',2);
plot(t,out.Kthrust/100);
plotstates();
xlabel('time [s]'),ylabel('velocity [m/s]');
legend('q1d','q1d filtfiltered',...
    ...'CoMxd','CoMxd filtered','CoMxd filtfiltered',...
    ...'vplot','vplot filtered','vplot filtfiltered',...
    'v reference','v actual',...
    'Kthrust/100',...
    'states')
grid on

% New: socket plots
figure(socketDisplacements),clf
plot(t,out.q(:,[j_socket_ext j_socket_rot]));
xlabel('time [s]'),ylabel('displacement [m or rad]');
grid on

figure(socketForces),clf
plot(t,out.tau(:,[j_socket_ext j_socket_rot]));
xlabel('time [s]'),ylabel('load [N or Nm]');
grid on


%% animation
% add ground tiles for walking range:
robot.appearance.base{2}(1,2) = ceil(10*max(out.q(:,1)))/10 + 0.5;

% threadmill effect:
q_anim = out.q;
v_tm = 1/(q_anim(:,1)\t); % option 2: use resulting ambulation speed
% q_anim(:,1) = q_anim(:,1)-v_tm*t; % uncomment to activate

% show animation:
showmotion(robot,out.tout,q_anim');

%% record animation (uncomment)
% l_leg = body.l_thigh + body.l_shank + body.y_foot;
% % create virtual bodyparts to prevent custom zooming at frame creation:
% robot.appearance.body{1} = [{'colour'},[0 0 0],...
%     {'sphere'}, [-l_leg-0.1 -0.1 0], 0.01,...
%     {'sphere'}, [l_leg+0.1  -0.1 0], 0.01,...
%     {'sphere'}, [0 l_leg*2+0.1 0], 0.01];
% robot.camera.zoom = 1.6;
% % to track the whole system (keep average centered)
% % try robot.camera = rmfield(robot.camera,'body');catch;end
% % try robot.camera = rmfield(robot.camera,'trackpoint');catch;end
% % try robot.camera = rmfield(robot.camera,'locus');catch;end
% % robot.appearance.body{1}{4}(1) = 0;
% % robot.appearance.body{1}{7}(1) = 0;
% % robot.camera.zoom = 1.2;
% 
% % to track the trunk (keep hip joint centered) instead 
% % robot.camera.body = 3;
% % robot.camera.trackpoint = [0 0 0];
% % robot.camera.locus = [0 0];
% 
% % to track the trunk x travel (keep only x centered) instead
% robot.camera.body = 1;
% robot.camera.trackpoint = [0 l_leg 0];
% robot.camera.locus = [0 0];
% 
% % Prepare the video file.
% vidObj = VideoWriter('anim.avi'); % movie name.
% vidObj.FrameRate = 30; % fps
% open(vidObj);
% 
% % interpolate data 
% t_vid = 0:1/vidObj.FrameRate:t_end;
% q_vid = interp1(t,out.q,t_vid);
% 
% % Create an animation.
% set(figure(1),'pos',[0 0 1280 720]+100); % 720p 19:6
% % axis tight manual
% % set(gca,'nextplot','replacechildren');
% for iF = 1:length(t_vid)
%     % create frame
%     showmotion(robot,[0 1],([1;1]*q_vid(iF,:))');
%     % write frame to the file
%     currFrame = getframe(figure(1));
%     writeVideo(vidObj,currFrame);
% end
% % Saves file
% close(vidObj);
