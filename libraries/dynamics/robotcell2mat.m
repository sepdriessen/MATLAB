function robot = robotcell2mat(robot)
% robot = robotcell2mat(robot)
% Convert a spatial v2 robot model to one without cell arrays, 
% to be compatible with Simulink (R2020b) Matlab functions.
% Appearance is deleted.

% determine 2D or 3D
n = size(robot.Xtree{1},1); % n=3 means 2D, n=6 means 3D

% function for converting cell to matrices matrices
cell3D=@(mycell)reshape(cell2mat(mycell),n,n,robot.NB);

robot.Xtree=cell3D(robot.Xtree);
robot.I=cell3D(robot.I);

% special operation for joint types (first add z to 1-char joint types)
robot.jtype=cell2mat(cellfun(...
    @(id)[id repmat('z',1,2-length(id))],robot.jtype','uniformoutput',false));

% remove appearance if exists
if isfield(robot,'appearance')
   robot = rmfield(robot,'appearance');
end

