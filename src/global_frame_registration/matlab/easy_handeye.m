% We solve AX = ZB
% where
% A = {Marker pose wrt camera}
% B = {EE wrt Robot base (PSM1)}
% X = {EE to marker}
% Z = {Robot base wrt camera}

% NB :  quaternion vector of the form q = [w x y z],
% tf_echo <source_frame> <target_frame>
%   getTransform(tftree,targetframe,sourceframe)
clear all
close all
clc

% roslaunch aruco_ros easy_handeye.launch
%%
rosinit;
tftree = rostf;
pause(1);
tftree.AvailableFrames;

% we dont need callback! just read once.
%arucoPose = rossubscriber('/aruco_single_left/pose');
%poses = receive(arucoPose,10);

% camera_frame = 'camera_link';
% camera_frame = 'camera_color_optical_frame';
camera_frame = 'endoscope/right_camera';
samples = 10;

A = zeros(4*samples,4);
Y = zeros(4*samples,4);

Hmarker2world = zeros(4,4,samples);
Hgrid2cam = zeros(4,4,samples);

for i=0:samples-1
    log_sample = sprintf('Sample number %d',i);
    disp(log_sample)
    
    waitForTransform(tftree, 'aruco_marker_frame',camera_frame);
    Marker_tf = getTransform(tftree, 'aruco_marker_frame',camera_frame);
    Marker = quat2tform([Marker_tf.Transform.Rotation.W, Marker_tf.Transform.Rotation.X, Marker_tf.Transform.Rotation.Y, Marker_tf.Transform.Rotation.Z]);
    Marker(1:3,4) = [Marker_tf.Transform.Translation.X, Marker_tf.Transform.Translation.Y, Marker_tf.Transform.Translation.Z];
%     Marker = inv(Marker);
    %Marker2World =  eye(4);
    %Marker2World(1:3,1:3) = rotz(180);
    
    ECM2World_tf = getTransform(tftree,'ECM','world');
    ECM_2World = quat2tform([ECM2World_tf.Transform.Rotation.W, ECM2World_tf.Transform.Rotation.X, ECM2World_tf.Transform.Rotation.Y,ECM2World_tf.Transform.Rotation.Z]);
    ECM_2World(1:3,4) = [ECM2World_tf.Transform.Translation.X,ECM2World_tf.Transform.Translation.Y,ECM2World_tf.Transform.Translation.Z];
    
    % EE2World_tf = getTransform(tftree, 'world','PSM1');
    % EE2World = quat2tform([EE2World_tf.Transform.Rotation.W, EE2World_tf.Transform.Rotation.X, EE2World_tf.Transform.Rotation.Y,EE2World_tf.Transform.Rotation.Z]);
    % EE2World(1:3,4) = [EE2World_tf.Transform.Translation.X,EE2World_tf.Transform.Translation.Y,EE2World_tf.Transform.Translation.Z];
    
    %ECM2ECM_base_tf = getTransform(tftree, 'ECM_base','ECM');
    %ECM2ECM_base = quat2tform([ECM2ECM_base_tf.Transform.Rotation.W, ECM2ECM_base_tf.Transform.Rotation.X, ECM2ECM_base_tf.Transform.Rotation.Y,ECM2ECM_base_tf.Transform.Rotation.Z]);
    %ECM2ECM_base(1:3,4) = [ECM2ECM_base_tf.Transform.Translation.X,ECM2ECM_base_tf.Transform.Translation.Y,ECM2ECM_base_tf.Transform.Translation.Z];
%     
    World2Marker_tf = getTransform(tftree, 'world', 'aruco_marker_frame');
    World2Marker = quat2tform([World2Marker_tf.Transform.Rotation.W, World2Marker_tf.Transform.Rotation.X, World2Marker_tf.Transform.Rotation.Y,World2Marker_tf.Transform.Rotation.Z]);
    World2Marker(1:3,4) = [World2Marker_tf.Transform.Translation.X,World2Marker_tf.Transform.Translation.Y,World2Marker_tf.Transform.Translation.Z];
    
    A(4*i+1,:) = Marker(:,1);
    A(4*i+2,:) = Marker(:,2);
    A(4*i+3,:) = Marker(:,3);
    A(4*i+4,:) = Marker(:,4);
    
    Y(4*i+1,:) = ECM_2World(:,1);
    Y(4*i+2,:) = ECM_2World(:,2);
    Y(4*i+3,:) = ECM_2World(:,3);
    Y(4*i+4,:) = ECM_2World(:,4);

%     dual quaternion setting parameters --- OLD
    Hmarker2world(:,:,i+1) = World2Marker*Marker;
    Hgrid2cam(:,:,i+1) = ECM_2World  ;

    disp('Press Enter to continue...');
    waitforbuttonpress;    
end

rosshutdown;

%%
% A = Marker;
% Y = ECM_2World;
% 
% X = inv(A(1:4,:)) * Y(1:4,:);
% X = X * World2Marker;

%X = (inv(A'* A) *A'* Y)';
%X = X * World2Marker;


% % 
% x_quat = rotm2quat(X(1:3,1:3));
% x_trasl = X(1:3,4);
% static_transform = [x_trasl' x_quat(2:4) x_quat(1)] % in ros format
% static_transform_publisher x y z qx qy qz qw frame_id child_frame_id  period_in_ms

%% 
% Ra Rx = Rz Rb
% Ra tx+ ta= Rz tb + tz

[Hcam2marker_, err] = hand_eye_dual_quaternion(Hmarker2world, Hgrid2cam);
% 
x_quat = rotm2quat(Hcam2marker_(1:3,1:3));
x_trasl = Hcam2marker_(1:3,4);
static_transform = [x_trasl' x_quat(2:4) x_quat(1)]

% ENDOSCOPE
% 5  samples 0.0026   -0.0000   -0.0017   -0.0001   -0.0004    0.0084    1.0000 
% 20 samples 0.0024   -0.0000    0.0032   -0.0000    0.0002    0.0005    1.0000

% % 
% r2w = [0.0242370567028 -0.00229869138384 -0.0161744780611];
% psm2w=  [0.004, -0.008, 0.000];
% d = sqrt( (r2w(1) - psm2w(1))^2 + (r2w(2) - psm2w(2))^2 + (r2w(3) - psm2w(3))^2)

    
    
    

   
   
% 22 JAN 
% LEFT  0.0025   -0.0001   -0.0014   -0.0004    0.0001    0.0015    1.0000 
% RIGHT 0.0025   -0.0001   -0.0004   -0.0003   -0.0001   -0.0001    1.0000


    
