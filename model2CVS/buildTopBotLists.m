% ==========================================================
% Copyright (C) Damien Berget 2013
% This code is only usable for non-commercial purpose and 
% provided as is with no guaranty of any sort
% ==========================================================
% 
% Matlab STL Slicer step 1.
% See http://exploreideasdaily.wordpress.com for details.
function [triBottomList, triTopList] = buildTopBotLists(vertices, tessellation)

%find min and maz z for each triangle
%Finding the lower Z vertex and the higher Z vertex and sorting them.

%Ztri is storing columwise the 3rd column values from the variable  'vertices'. It is storing the value of the first vertex in one column, second vertex in the second, and third vertex in the third.
%ZTri = [the 3rd column value for all first vertices | the 3rd column values for all second vertices | the 3rd column values for all third vertices]
ZTri = [vertices(tessellation(:,1),3) vertices(tessellation(:,2),3) vertices(tessellation(:,3),3)];
minZ = min(ZTri,[], 2);
maxZ = max(ZTri,[], 2);

%build top & bottom list
[val, idx] = sort(minZ);  %idx is the index value corresponding to the value in min Z
triBottomList = [val idx];
[val, idx] = sort(maxZ);
triTopList = [val idx];
