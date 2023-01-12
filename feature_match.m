function [match_list] = feature_match(u1,v1,im1,u2,v2,im2,window_size)
%
%   [match_list] = match_features(…)
%
%   input:
%       ui,vi: feature coordinates
%       imi: images
%       window_size: size for ssd matching
%   output:
%       match_list: nx5 of n matching pairs
%       each row [u1,v1,u2,v2,err]
%           holds coordinates and matching error (ssd)

feature_list1(:,1) = u1(:);
feature_list1(:,2) = v1(:);

feature_list2(:,1) = u2(:);
feature_list2(:,2) = v2(:);

%% init
im_dim1 =size(im1);
im_dim2 =size(im2);

match_list =[];

s = size (feature_list1);
n1 = s(1);
s = size (feature_list2);
n2 = s(1);

dist_thresh = 5;

win_rad = floor(window_size/2);
w = [-win_rad:win_rad];
match_count = 1;

% for each feature in the first list
for i1 = 1:1:n1
    
    x1 = feature_list1(i1,1);
    y1 = feature_list1(i1,2);
    
    %fprintf('\n%4d p1 %3d/%3d',i1,x1,y1);
    
    match1  = -1;
    match2  = -1;
    minsum1 = 10000000;
    minsum2 = 10000000;
    
    % match against all features in the second list
    for i2 = 1:1:n2
        
        x2 = feature_list2(i2,1);
        y2 = feature_list2(i2,2);
        
        % zssd over correlation window
        sum = 0;
        
        %sqrt((x1-x2)^2+(y1-y2)^2) < dist_thresh && ...
        if ( ...
                sqrt((x1-x2)^2+(y1-y2)^2) < dist_thresh && ...
                y1>win_rad && y1<im_dim1(1)-win_rad+1 &&...
                x1>win_rad && x1<im_dim1(2)-win_rad+1 &&...
                y2>win_rad && y2<im_dim2(1)-win_rad+1 &&...
                x2>win_rad && x2<im_dim2(2)-win_rad+1 ...
                )
            
            % calc similarity
            sum = zssd(im1(y1+w,x1+w),im2(y2+w,x2+w));
            %sum = zncc(im1(y1+w,x1+w),im2(y2+w,x2+w));
            % is this match better than the second best match???
            if sum < minsum2
                % yes, exchange
                minsum2 = sum;
                match2 = i2;
            end
            
            % is it also better than the best match ??
            if sum < minsum1
                % we found a new best match
                minsum2 = minsum1;
                match2 = match1;    % former best is now second best !
                minsum1 = sum;
                match1 = i2;
            end
        end %if
        
    end %for
    
    % check peak ratio of best and second best match
    if (minsum2>0 && minsum2*0.6>minsum1)
        % second best is far enough away
        
        % save best match
        mc = match_count;
        match_list(mc,1) = x1;
        match_list(mc,2) = y1;
        if(match1>0)
            match_list(mc,3) = feature_list2(match1,1);
            match_list(mc,4) = feature_list2(match1,2);
            match_list(mc,5) = minsum1;
            %fprintf(' matches to p2 %3d/%3d with %d/%d q=%f', feature_list2(match1,1),feature_list2(match1,2),minsum1, minsum2,minsum1/minsum2);
        else
            match_list(mc,3) = -1;
            match_list(mc,4) = -1;
            match_list(mc,5) = -1;
        end
        match_count = match_count +1;
    end %if
    
end %for