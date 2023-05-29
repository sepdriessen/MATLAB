% startup file to load libraries

addpath(genpath('../libraries'));
if exist('showmotion.m','file')==2 && exist('MoI_2D_line.m','file')==2
    disp(['Library successfully loaded. Current folder: ' pwd])
else
    disp(['WARNING: Library not found. Current folder: ' pwd])
end
