%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%file name: Main_StereoVisualOdometry.m
%authors :  Kiran Bhat, Meheboob Khan
%Project :  Visual Odometry
%Version :  1.0
%Date    :  08-08-2021
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
clear all;
close all;

%% %%=========================Fetch Inputs===========================%%%%%%%%%
% workingDirL = 'D:\Fh-Dortmund\ComputerVision\Project\NewWorkspace\img_undistort\left\';
% workingDirR = 'D:\Fh-Dortmund\ComputerVision\Project\NewWorkspace\img_undistort\right\';

DirecLeft = 'E:\Code\Code\Images\raw_images\Left_images\';
DirecRight = 'E:\Code\Code\Images\raw_images\Right_images\';

Left_Images = dir(fullfile(DirecLeft,'*.png'));
Right_Images = dir(fullfile(DirecRight,'*.png'));
Left_Images = {Left_Images.name}';
Right_Images = {Right_Images.name}';
distanceMatrix = [];
QuaterArray =zeros(101,4);
TranslationArray =zeros(101,3);
Terror = zeros(101,1);
i = 1;
% start loop
for loop = 1:248
    
    LImage = imread(strcat(DirecLeft,Left_Images{loop}));
    RImage = imread(strcat(DirecRight,Right_Images{loop}));
    %% %%=========================Assign Intrinsic and extrinsic Parameters===========================%%%%%%%
    width = 640;
    height= 480;
    
    %% Left camera
    focalX_left = 203.86763661;
    focalY_left = 203.11430305;
    principal_point_lu = 319.31807059;
    principal_point_lv = 243.13746003;
    IntrinsicCamL = [focalX_left, 0, principal_point_lu; 0, focalY_left, principal_point_lv; 0, 0, 1];
    
    %% Right camera
    focalX_right = 205.37722847;
    focalY_right = 206.02343888;
    principal_point_ru = 305.79383101;
    principal_point_rv = 233.63159936;
    IntrinsicCamR = [focalX_right, 0, principal_point_ru; 0, focalY_right, principal_point_rv; 0, 0, 1];
    
	Intr_1 =[203.86763661, 0., 319.31807059;
            0., 203.11430305, 243.13746003;
            0., 0., 1.]';
        
    camPar_1 = cameraParameters('IntrinsicMatrix',Intr_1);
    
    Intr_2 =[205.37722847, 0., 305.79383101;
                0., 206.02343888, 233.63159936;
                0., 0., 1.]';

    camPar_2 = cameraParameters('IntrinsicMatrix',Intr_2);
    
    rotationMatrix = [ 0.9998053017199768, 0.011197738450911482, 0.016247132245484134;
                -0.011147758116323998, 0.9999328574031366, -0.0031635699090552883;
                -0.016281466199246437, 0.00298183486707869, 0.9998630018753666 ]';
            
    translationMatrix = [ -0.07961594300469246, 0.0007443452072558462, 0.0004425529195268342 ]';
    
    Pose_LR = [ 0.9998053017199768, 0.011197738450911482, 0.016247132245484134, -0.07961594300469246;
                    -0.011147758116323998, 0.9999328574031366, -0.0031635699090552883, 0.0007443452072558462;
                    -0.016281466199246437, 0.00298183486707869, 0.9998630018753666, 0.0004425529195268342;
                    0.0, 0.0, 0.0, 1.0];
                
    Pose_RL = inv(Pose_LR);
    
    stereoParams = stereoParameters(camPar_1,camPar_2,rotationMatrix,translationMatrix);

    [LeftImg, RightImg] = rectifyStereoImages(LImage,RImage,stereoParams,'OutputView', 'valid');
    
    stereo_baseline = abs(Pose_RL(1, 4));
    
    figure()
    imshowpair(LImage,RImage,'blend','Scaling','joint')
    title("Left and right blended images");
    
    % %% ========================= Rectified images visualization========================
    h1 = figure;
    set(h1, 'NumberTitle', 'Off', 'Position', [10, 10, 1200, 500]);
    set(h1, 'Name', 'Rectified');
    subplot(1, 2, 1); imshow(LeftImg); title('Rectified Left Image', 'FontWeight', 'Bold');
    subplot(1, 2, 2); imshow(RightImg); title('Rectified Right Image', 'FontWeight', 'Bold');
    
    figure;
    idisp(stereoAnaglyph(LeftImg, RightImg));
    title('StereoAnaglyph', 'FontWeight', 'Bold');
    
    
    %%%=========================Fetching the features from the images=========================%% 
    
    LeftEdge = canny_edge(LeftImg);
    figure;
    idisp(LeftEdge);
    title('Canny Filter For left Image', 'FontWeight', 'Bold');
       
    figure();
    RightEdge = canny_edge(RightImg);
    idisp(RightEdge);
    title('Canny Filter For Right Image', 'FontWeight', 'Bold'); 
    
    [u2_l, v2_l]=harris_corner_dector(LeftImg);
    [u2_r, v2_r]=harris_corner_dector(RightImg);

    idisp(LeftImg, 'new');
    title('Harris filter on left Image', 'FontWeight', 'Bold'); 
    hold on;
    plot(u2_l, v2_l,'rs');
    hold off;
    
    idisp(RightImg, 'new');
    title('Harris filter on right Image', 'FontWeight', 'Bold'); 
    hold on;
    plot(u2_r, v2_r,'gs');
    hold off;

    %% %%========================= Matching the image features on both the frames=========================%% %%
    
    MatchedFeatures = feature_match(u2_l, v2_l, LeftImg, u2_r, v2_r, RightImg, 21);
    mfL =MatchedFeatures(:,1:2)';
    mfR =MatchedFeatures(:,3:4)';
 
    figure;
    stdisp(LeftImg,RightImg);
    hold on
    
    match_plot(LeftImg,RightImg,mfL',mfR');
    title('Matched Features', 'FontWeight', 'Bold');
    
    warningstate = warning('off','vision:ransac:maxTrialsReached');
    %Ramya [F,in,r] = ransac(@fmatrix, [mfL; mfR], 1e-4, 'verbose','maxTrials',10000,'maxDataTrials',1000);
    [F,in] = ransac(@fmatrix, [mfL; mfR], 1e-4, 'verbose','maxTrials',10000,'maxDataTrials',1000);

    warning(warningstate);
    mfLn =MatchedFeatures(in,1:2)';
    mfRn =MatchedFeatures(in,3:4)';
    [Fn,rmax] = fmatrix(mfLn, mfRn);
    
    figure;
    plot(mfL,mfR,'o');
    hold on
    modelInliers = polyfit(mfLn,mfRn,1);

    inlierPts = MatchedFeatures(in,1:2);
    hor = [min(inlierPts(:,1)) max(inlierPts(:,1))];
    vec = modelInliers(1)*hor + modelInliers(2);
    plot(hor, vec, 'g-')
    title(' RANSAC ', 'FontWeight', 'Bold');
    hold off
    
    %% %% ========================= Point Cloud visualization  =========================

    ul = MatchedFeatures(:, 1);
    ur = MatchedFeatures(:, 3);
    vl = MatchedFeatures(:, 2);
    
    row_size = size(ul, 1);
    point_index = 1;
    world_coordinates = [];
    u0=principal_point_lu;
    v0=principal_point_lv;

    for index = 1 : row_size
        dispar = ul(index, 1) - ur(index, 1);
        % Ignore the points whose disparity is 0 or negative
        if (dispar > 0)
            x = (stereo_baseline * (ul(index, 1) - u0)) / dispar;
            y = (stereo_baseline * (vl(index, 1) - v0)) / dispar;
            z = (focalX_left * stereo_baseline) / dispar;
            world_coordinates(point_index, 1) = x;
            world_coordinates(point_index, 2) = y;
            world_coordinates(point_index, 3) = z;
            point_index = point_index + 1;
        end
    end
    xx=world_coordinates(:,1);
    yy=world_coordinates(:,2);
    zz=world_coordinates(:,3);
    
    for count=1:length(xx)
        Lcolor = iread(strcat(DirecLeft,Left_Images{loop}));
        surface(xx(count), yy(count), zz(count), Lcolor, 'FaceColor', 'texturemap', ...
            'EdgeColor', 'none', 'CDataMapping', 'direct')
        xyzlabel
        set(gca,'ZDir', 'reverse'); set(gca,'XDir', 'reverse')
        hold on;
    end

    %To display disparity map
    LeftImg = iread(strcat(DirecLeft,Left_Images{loop}),'reduce', 2);
    RightImg = iread(strcat(DirecRight,Right_Images{loop}),'reduce', 2);
       
    status = ones(size(dispar));
    [U,V] = imeshgrid(LeftImg);
    status(isnan(dispar)) = 5;
    status(U<=90) = 2;
    status(similarity<0.8) = 3;
    status(peak.A>=-0.1) = 4;
    figure();
    idisp(status)
    colormap( colorname({'lightgreen', 'cyan', 'blue', 'orange', 'red'}) )
    ipixswitch(isnan(dispar), 'red', dispar/90);
    dispar = dispar + 274;
    u0 = size(LeftImg,2)/2;
    v0 = size(LeftImg,1)/2;
    X = stereo_baseline*(U-u0)./dispar;
    Y = stereo_baseline*(V-v0)./dispar;
    Z = focalX_left * stereo_baseline./dispar;
    shading interp; view(-150, 75)
    set(gca,'ZDir', 'reverse'); set(gca,'XDir', 'reverse')
    colormap(flipud(hot))
    colorbar;
    surface(X, Y, Z, LeftImg, 'FaceColor', 'texturemap', ...
        'EdgeColor', 'none', 'CDataMapping', 'direct')
    xyzlabel
    figure;
    idisp(LeftImg)
    plot_point(MatchedFeatures, 'y+', 'textcolor', 'y', 'printf', {'%.1f', Z});
    imshow(LeftImg,[-128 128]);
    colormap(gca,jet); % colormap jet;
    colorbar;
    title('Disparity Map', 'FontWeight', 'Bold');
    set(gca,'ZDir', 'reverse'); set(gca,'XDir', 'reverse')

    [LeftImg, RightImg] = rectifyStereoImages(LImage,RImage,stereoParams,'OutputView', 'valid');
    dispMap = disparity(LeftImg, RightImg,'BlockSize',15,'DisparityRange',[-128 128],'Method','BlockMatching');

    % 3D-Scene from Disparity Map(BM)
    points3D = reconstructScene(dispMap,stereoParams);
    points3D = points3D ./ 10;
    sceneImageGray = im2bw(LeftImg);
    blob = vision.BlobAnalysis('BoundingBoxOutputPort', true,'MinimumBlobAreaSource', 'Property','MinimumBlobArea', 200);
    [area,centroid,bbox] = step(blob, sceneImageGray);
    
    
    %% %% ========================= Find the 3-D world coordinates ========================= %% %% 
    centroidsIdx = sub2ind(size(dispMap), round(centroid(:, 2)), round(centroid(:, 1)));
    X = points3D(:, :, 1);
    X(isnan(X))=1.0;
    Y = points3D(:, :, 2);
    Y(isnan(Y))=1.0;
    Z = points3D(:, :, 3);
            A = [cos(pi/18) sin(pi/18) 0 0;
        -sin(pi/18) cos(pi/18) 0 0;
         0 0 1 0;
         5 5 10 1];
    tform1 = affine3d(A);
    ptCloud = pointCloud(points3D);
    figure;
    pcshow(ptCloud);
    title('Point Cloud', 'FontWeight', 'Bold');    
    L_imu_pose = [0.028228787368606456, -0.999601488301944, 1.2175294828553618e-05, 0.02172388268966517;
			  0.014401251861751119, -0.00041887083271471837, -0.9998962088597202, -6.605455433829172e-05;
			  0.999497743623523, -0.028225682131089447, 0.014407337010089172, -0.00048817563004522853;
			  0.0, 0.0, 0.0, 1.0];

    ptCloudTformed = pctransform(ptCloud,tform1);
    tform = pcregrigid(ptCloud,ptCloudTformed,'Extrapolate',true);
    temp = tform.T' * L_imu_pose;
    rotm = temp(1:3,1:3);
    EstTran = temp(1:3, 4)';
    Q = UnitQuaternion(rotm);
    QuaterArray(i,1)=Q.v(1);
    QuaterArray(i,2)=Q.v(2);
    QuaterArray(i,3)=Q.v(3);
    QuaterArray(i,4)=Q.s;
    
    Groundtran = [GroundtranslationMatrix(loop,1), GroundtranslationMatrix(loop,2), GroundtranslationMatrix(loop,3)];
    translationMatrixError =  sqrt(mean((Groundtran-EstTran).^2));
    translationMatrixArray(i,1) = EstTran(1);
    translationMatrixArray(i,2) = EstTran(2);
    translationMatrixArray(i,3) = EstTran(3);
    Terror(i,1) = translationMatrixError;

    centroids3D = [X(centroidsIdx)'; Y(centroidsIdx)'; Z(centroidsIdx)'];
    surface(X, Y, Z, LeftImg, 'FaceColor', 'texturemap', 'EdgeColor', 'none', 'CDataMapping', 'direct')

    % Find the distances from the camera in meters.
    dists = sqrt(sum(centroids3D .^ 2));
    distanceMatrix{i} = dists;
    i = i + 1;
end
%% %% <<<<<<<<<<<<<<<<<<<<<<<<<  GroundTruth Reconstruction >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>...
sc = trackingScenario();
groundtruth(1,:) = [];
final = platform(sc);

traj = waypointTrajectory(groundtruth(:,2:4),'TimeOfArrival',groundtruth(:,1),'Orientation',quaternion(groundtruth(:,5:7), 'euler', 'XYZ', 'frame'));
tInfo = waypointInfo(traj);
final.Trajectory = traj;
r = record(ts);
Tposes = [r(:).Poses];
Tposition = vertcat(pposes.Position);
tp = theaterPlot('XLim',[0 30], 'YLim', [-8 8]);
trajPlotter = trajectoryPlotter(tp, 'DisplayName', 'Trajectory');
plotTrajectory(trajPlotter, {Tposition})



