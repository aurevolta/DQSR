function []=get_DQSR_template()

% take onesat_S
file2copy = which('onesat_S.m');

% copy in current directory
copyfile(file2copy,[cd,filesep,'template.m'])

% open editor
edit template



end