%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% function [u,v] = harris_corner_dector(im)
% Applies harris corner detection algorithm to the given image
%  img   - given image matrix
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function [u,v] = harris_corner_dector(im)

im = im(:,:,1);
sigma = 1;
radius = 1;
order = (2*radius +1);
threshold = 1000;

%derivatives in x and y direction
[dx, dy] = meshgrid(-1:1, -1:1);

Ix = conv2(double(im), dx, 'same');
Iy = conv2(double(im), dy, 'same');

%Gaussian filter
dim = max(1, fix(6*sigma));
m=dim; n = dim;

[h1, h2] = meshgrid(-(m-1)/2: (m-1)/2, -(n-1)/2: (n-1)/2);
hg = exp(-(h1.^2+h2.^2)/(2*sigma^2));
[a, b] = size(hg);
sum = 0;
for i = 1:a
    for j=1:b
        sum = sum + hg(i,j);
    end
end

g = hg ./sum;

%calculate entries of the M matrix
Ix2 = conv2(double(Ix.^2), g, 'same');
Iy2 = conv2(double(Iy.^2), g, 'same');
Ixy = conv2(double(Ix.*Iy), g, 'same');

% Harris measure
R = (Ix2.*Iy2 - Ixy.^2) ./ (Ix2+Iy2+eps);

% find local maxima
mx = ordfilt2(R, order^2, ones(order));

harris_points = (R == mx) & (R > threshold);

[rows, cols] = find(harris_points);
u = rows;v=cols;
end
