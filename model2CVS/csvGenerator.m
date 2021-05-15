clear all; close all; clc;

model = 'pentagon.stl';
[allTargets, sz] = startSlicer(model);

%Removing all the duplicate coordinates that enter the system
Z0 = unique(allTargets{1},'rows'); 
Z1 = unique(allTargets{2},'rows');
Z2 = unique(allTargets{3},'rows');

allTarget = {Z0 Z1 Z2};
csvwrite('morphGoalPoint-triangle.csv',allTarget)
