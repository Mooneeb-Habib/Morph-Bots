clear all; close all; clc;

model = 'cube-side-125.stl';
[allTarget, sz] = startSlicer(model);
display(allTarget{1})

csvwrite('morphGoalPoint.csv',allTarget)