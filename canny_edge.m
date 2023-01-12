%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% function emap = canny_edge(img)
% Applies canny edge detection algorithm to the given image
%  img   - given image matrix
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function emap = canny_edge(img)

% Constant sigma fsize, tlow and thigh
sigma = 1; %  sigma - the value of sigma for the derivative of gaussian blur
fsize = 5; %  fsize - given size of the gaussian blur kernal to be applied
tlow = 10; %  tlow  - given lower threshold, used for hysteresis to avoid streaking
thigh = 20;%  thigh - given upper threshold, anything above which is an edge pixel

% Convert image to double (0-1)
img = double(img);

% Blur image using gaussian blur kernal function
blur = ifilter_gauss(img, fsize);

% Get magnitude and direction using mag_dir function
[temp_mag,temp_dir] = mag_dir(blur);

% Use non-maximal suppression to get thin edges
edgeMap = thinEdges(temp_mag,temp_dir);

% Finally use hysteresis to avoid streaking in final image
emap = hystereis(edgeMap, tlow, thigh);


%% Magnitude and Direction %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Applies appropriate equations to retieve matricies of magnitude and
% direction
    function [mag, dir] = mag_dir(fimg)
        fimg = padarray(fimg,[1,1], 'replicate','post');

        % Applies matrix values to get dy/dx
        dy = fimg(:,2:end) - fimg(:,1:end-1);
        dy = padarray(dy,[0,1], 'replicate','post');
        dx = fimg(2:end,:) - fimg(1:end-1,:);
        dx = padarray(dx,[1,0], 'replicate','post');

        dir = atan2d(dx,dy);
        mag = sqrt((dy.^2) + (dx.^2));

        % Applies roundUp function to the direction matrix
        dir = roundUp(dir);
    end

% Rounds up the angle to either 0, 45, 90, or 135
    function dir = roundUp(dir)
        dir(dir < 22.5 & dir >= -22.5) = 0;
        dir(dir < 67.5 & dir >= 22.5 | dir >= -67.5 & dir < -22.5) = 45;
        dir(dir < 112.5 & dir >= 67.5 | dir >= -112.5 & dir < -67.5) = 90;
        dir(dir < 157.5 & dir >= 112.5 | dir >= -157.5 & dir < -112.5) = 135;
        dir(dir >= 157.5 | dir < -112.5) = 0;
    end

%% Non-Maxmal Suppression %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Finds definite edge pixels by comparing candidate pixel to its neighbours
    function edgeMap = thinEdges(mag, dir)
        [x, y] = size(mag);
        edgeMap = mag;
        % For-loop to iterate through size of image, with 1 pixel padding
        for i=2:x-1
            for j=2:y-1
                % Find co-ordinates of the next and prev pixels depening
                % on the direction of the gradient
                switch(dir(i,j))
                    case 0
                        prevPixel = mag(i,j-1);
                        nextPixel = mag(i,j+1);
                    case 45
                        prevPixel = mag(i-1,j-1);
                        nextPixel = mag(i+1,j+1);
                    case 90
                        prevPixel = mag(i-1,j);
                        nextPixel = mag(i+1,j);
                    case 135
                        prevPixel = mag(i+1,j-1);
                        nextPixel = mag(i-1,j+1);
                    otherwise
                        error('Invalid Direction');
                end
                % If either neighbours are bigger, turn off candidate pixel
                if(mag(i,j) <= prevPixel || mag(i,j) < nextPixel)
                    edgeMap(i,j) = 0;
                end
            end
        end
    end

%% Hysterisis %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Compares candidate pixel to thresholds to ensure it is an edge pixel, if
% it is not above thigh, but it is above tlow, see if its neighbours are
% definite edge pixels, if they are, candidate becomes definite
    function emap = hystereis(edgeMap, tlow, thigh)

        % Get all pixels in edge map that are above the lower threshold
        aboveLower = edgeMap > tlow;
        % Get Co-ordinates of the pixels that are above the upper threshold
        [aboveUpper_X, aboveUpper_Y] = find(edgeMap > thigh);
        % Return emap of all points in aboveLower which are connected to a
        % point above the upper threshold
        emap = bwselect(aboveLower, aboveUpper_Y, aboveUpper_X);
    end

end