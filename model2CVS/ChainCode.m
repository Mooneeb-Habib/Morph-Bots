function [Goal,sz] = ChainCode(slices)

Goal = {};
for i = 1:length(slices)
    
    fig = slices{i};
    
    %The number of samples to subsample a boundary by
    No_of_samples = 8;
    
    %The amount by which to reduce the size of the image
    divisor = 10;
    
    %Performing Otsu-Thresholding
    otsu = graythresh(fig);
    bin_stroke = imbinarize(fig, otsu);
    
    % Extracting the boundaries of the segmented regions
    bound = boundaries(bin_stroke);
    
    b = bound{1}; % The only region of interest is the first region
    
    [M, fig] = size(bin_stroke);
    % Creating image using extracted boundary pixels
    thin_bound = bound2im(b, M, fig, min(b(:, 1)), min(b(:, 2)));
    figure, imshow(thin_bound); xlabel('Thin Stroke boundaries');
    
    % Sub-sampling extracted boundary pixels for smaller chain codes
    [subsamp_bound, ~] = bsubsamp(b, No_of_samples);
    
    % Creating image using sub-sampled extracted boundary pixels
    subsamp_bound_img = bound2im(subsamp_bound, M, fig, min(subsamp_bound(:, 1)), min(subsamp_bound(:, 2)));
    figure, imshow(subsamp_bound_img); xlabel('Subsampled Stroke boundaries');
    
    %Inverting the emphasis of focus to the foreground
    inverted_img = imcomplement(subsamp_bound_img);
    
    %Finding the rows and columns of the object in each slice
    [row, col] = find(inverted_img == 0);
    
    %Adjusting the size of each slice to fit the code
    row = round(row./divisor);
    col = round(col./divisor);
    
    %The Coordinates of each slice
    sliceCoords = [row col];
    Goal{end+1} = sliceCoords;
end
    %The size of the image
    sz = length(subsamp_bound_img)./divisor;
end

