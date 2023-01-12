function [im_out] =ifilter_gauss(im, N)
% implementation without matrix algebra

%% init
im_dim =size(im);
im_out =zeros(im_dim);

%% calculate gaussian kernel
sigma =N/5;     % kernel size N must be odd
hwin =(N-1)/2;  % half kernel size
ic =hwin+1;     % center index

K =zeros(N,N);
for iu=1:N
    for iv=1:N
        K(iv,iu) =exp(-((iu-ic)^2+(iv-ic)^2)/(2*sigma^2));
    end
end
K =K./(2*pi*sigma^2);
        

%% calculate 2D filter
for iu=ic:1:im_dim(2)-hwin
    for iv=ic:1:im_dim(1)-hwin
       
        % calculate convolution (sum)
        y =0;
        for du=-hwin:hwin
            for dv=-hwin:hwin
                % get kernel element
                k =K(dv+hwin+1, du+hwin+1);
                % sum up
                y = y + k * im(iv+dv,iu+du);
            end
        end

        % store result in output image
        im_out(iv,iu) = y;
    end
end
