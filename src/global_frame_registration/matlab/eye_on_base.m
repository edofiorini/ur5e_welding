clc;
clear all;
close all;

rosinit;
tftree = rostf;
pause(1);
tftree.AvailableFrames;

waitForTransform(tftree, 'aruco_marker_frame','endoscope/left_camera');
Marker_tf = getTransform(tftree, 'aruco_marker_frame','endoscope/left_camera');
Marker = quat2tform([Marker_tf.Transform.Rotation.W, Marker_tf.Transform.Rotation.X, Marker_tf.Transform.Rotation.Y, Marker_tf.Transform.Rotation.Z]);
Marker(1:3,4) = [Marker_tf.Transform.Translation.X, Marker_tf.Transform.Translation.Y, Marker_tf.Transform.Translation.Z];

ECM2World_tf = getTransform(tftree,'ECM','world');
ECM_2World = quat2tform([ECM2World_tf.Transform.Rotation.W, ECM2World_tf.Transform.Rotation.X, ECM2World_tf.Transform.Rotation.Y,ECM2World_tf.Transform.Rotation.Z]);
ECM_2World(1:3,4) = [ECM2World_tf.Transform.Translation.X,ECM2World_tf.Transform.Translation.Y,ECM2World_tf.Transform.Translation.Z];

% World2Marker_tf = getTransform(tftree, 'world', 'aruco_marker_frame');
% World2Marker = quat2tform([World2Marker_tf.Transform.Rotation.W, World2Marker_tf.Transform.Rotation.X, World2Marker_tf.Transform.Rotation.Y,World2Marker_tf.Transform.Rotation.Z]);
% World2Marker(1:3,4) = [World2Marker_tf.Transform.Translation.X,World2Marker_tf.Transform.Translation.Y,World2Marker_tf.Transform.Translation.Z];
World2Marker = eye(4);
% World2Marker(3,4) = -0.005;
rosshutdown;

A = Marker;
Y = ECM_2World;

X = A * Y;
X = X * World2Marker;

fix_rot = rotz(180) *(rotx(90) * X(1:3,1:3)) ;

% x_quat = rotm2quat(X(1:3,1:3));
x_quat = rotm2quat(fix_rot);
x_trasl = X(1:3,4);
static_transform = [x_trasl' x_quat(2:4) x_quat(1)]
