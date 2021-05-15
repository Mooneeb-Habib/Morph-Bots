clear all; close all; clc;

model = 'pentagon.stl';
extrusionp = 350;
[allTargets, sz] = startSlicer(model,extrusionp);

%Removing all the duplicate coordinates that enter the system
Z0 = unique(allTargets{2},'rows'); 
Z1 = unique(allTargets{3},'rows');
Z2 = unique(allTargets{4},'rows');

allTarget = {Z0 Z1 Z2};
csvwrite('morphGoalPoint-pentagon.csv',allTarget)
