Legsim contains Matlab and Simulink implementations of a hybrid dynamic 2D model for design and development of lower limb prostheses.
They serve as supplementary material of Driessen et al. (2023), "A reduced-order closed-loop hybrid dynamic model for design and development of lower limb prostheses", Wearable Technologies, DOI: 10.1017/wtc.2023.6. Since its publication, several parts of code have been optimized, which might result in small numerical deviations.

INSTRUCTIONS
1) Run startup.m to load the "libraries" directory, which should include "dynamics/MoI_2D_line.m", "dynamics/robotcell2matt.m" and "spatial_v2" (Copyright (C) 2012 Roy Featherstone <http://royfeatherstone.org>, GNU General Public License v3.0). If the libraries directory is moved, startup.m should be modified accordingly.
2) Run either of 3 scripts to run their respective simulations and show animations:
- Leg_Script_VariableDamping.m
- Leg_Script_Socket_VariableDamping.m
- Leg_Script_Powered.m

COMPATIBILITY NOTE
Files are created in Matlab and Simulink R2020b. Forward compatibility has been verified for Matlab and Simulink R2023a.

LEGAL NOTE
LegSim is Copyright (C) 2023 Josephus J. M. Driessen

This program is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, version 3.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program. A plain-text version <documentation/gpl.txt> is
included in the distribution. If not, see <http://www.gnu.org/licenses/>.

Simulink (.slx) files contain internal Matlab functions, some of which are inspired from or modifications of files of spatial_v2 (Copyright (C) 2012 Roy Featherstone <http://royfeatherstone.org>),
which is also under the GNU General Public License v3.0 license:
*.slx/system/HDgc/transcalc
*.slx/system/HDgc/HD
*.slx/system/HDgc/gcPosVel
*.slx/system/HDgc/gcontact
*.slx/system/HDgc/Fpt

CONTENTS LegSim
filly.m
Leg_Model.m
Leg_Model_Socket.m
Leg_Pars_Body.m
Leg_Pars_Ctrl_VariableDamping
Leg_Pars_Device_Powered.m
Leg_Pars_Device_VariableDamping.m
Leg_Pars_Human.m
Leg_ParsCtrl_Powered.m
Leg_Script_Powered.m
Leg_Script_Socket_VariableDamping.m
Leg_Script_VariableDamping.m
Leg_Sim_Powered.slx
Leg_Sim_Socket_VariableDamping.slx
Leg_Sim_VariableDamping.slx
README.txt
startup.m

DEPENDENCIES (libraries)
dynamics/MoI_2D_line.m
dynamics/robotcell2matt.m
spatial_v2 (Copyright (C) 2012 Roy Featherstone <http://royfeatherstone.org>, GNU General Public License v3.0)

