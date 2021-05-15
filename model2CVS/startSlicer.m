function [Goal,sz] = startSlicer(stl_file,extrusion)

% ==========================================================
% Copyright (C) Damien Berget 2013
% This code is only usable for non-commercial purpose and
% provided as is with no guaranty of any sort
% ==========================================================
%
% Matlab STL Slicer step 1.
% See http://exploreideasdaily.wordpress.com for details.

dbstop if error

fileName = stl_file;

[vertices, tessellation] = readStl(fileName);

layerThickness = extrusion/3;

%build top amd bottom Z triangle list
[triBottomList, triTopList] = buildTopBotLists(vertices, tessellation);

%for display get limits
axis_increase = 25;
xLimits = [min(vertices(:,1))-axis_increase max(vertices(:,1))+axis_increase];
yLimits = [min(vertices(:,2))-axis_increase max(vertices(:,2))+axis_increase];

currZ = triBottomList(1,1);

%Index in the top/bot list
botIdx = 0; topIdx = 0;
%list of currently 'active' triangles
currTri = [];
%Array of all slices
slices = {};
%flag to skip the first iteration
flag = false;
%go through all the Z plans
hdl = subplot(1,1,1);
%axis off
while currZ <= triTopList(end, 1)
    figure;
    %add triangle upto currZ (from bottom list)
    if botIdx < numel(triBottomList)
        while triBottomList(botIdx + 1, 1) <= currZ
            currTri(end + 1) = triBottomList(botIdx + 1, 2);
            botIdx = botIdx + 1;
            if botIdx == size(triBottomList,1)
                break;
            end
        end
    end
    
    %remove triangle under currZ (from top list)
    if topIdx < numel(triTopList)
        remList = [];
        while triTopList(topIdx + 1, 1) < currZ
            remList(end + 1) = triTopList(topIdx + 1, 2);
            topIdx = topIdx + 1;
            if topIdx == size(triTopList,1)
                break;
            end
        end
        
        currTri = setdiff(currTri, remList);
    end
    
    %compute interections of all current triangles with current Z
    currIntersect = {};
    for idxTri = 1:numel(currTri)
        triCoo = vertices(tessellation(currTri(idxTri),:), :);
        currIntersect{end + 1} = triPlanIntersect(triCoo, currZ);
    end
    
    %display all the intersections
    cla(hdl)
    hold all
    axis equal
    set(gca,'xtick',[],'ytick',[])
    xlim(xLimits)
    ylim(yLimits)
    for idxObj = 1: numel(currIntersect)
        switch size(currIntersect{idxObj}, 1)
            case 1
                plot(currIntersect{idxObj}(:,1), currIntersect{idxObj}(:,2), '-k', 'MarkerSize', 2);
            case 2
                plot(currIntersect{idxObj}(:,1), currIntersect{idxObj}(:,2), '-k', 'MarkerSize', 2);
        end
    end
    title(sprintf('Slice Z = %4.2f', currZ))
    drawnow
    pause(0.2)
    
    %A check to not include the first blank image in the array
    if flag == true 
        %move to next plan
        currZ = currZ + layerThickness;
        %Saving the data of the figure
        save_fig = getframe;
        %Extracting the data of the figure as an image
        img_fig = save_fig.cdata;
        %Converting image to grayscale
        img_fig = im2bw(img_fig);
        %Removing border axis from the image
        img_fig(1:end,1:2) = 255; img_fig(end-1:end,1:end) = 255;
        img_fig = im2uint8(img_fig);
        %Inverting the image to make the object pixels white
        img_fig = imcomplement(img_fig);
        %Thickening the edges of each slice
        img_fig = imdilate(img_fig, strel('disk', 2));
        %Storing each slice
        slices{end+1} = img_fig;
    else
        flag = true;
    end
    
end

A = 1;
%Chain Code algorithm to convert the image into goal points.
[Goal,sz] = ChainCode(slices);
end

