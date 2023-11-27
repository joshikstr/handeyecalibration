% hand eye calibration
% determine the pose between the gripper of the Denso Cobotta robot and the
% ZED mini stereo camera (mounted above the gripper)
%
% B.Sc. Joshua Köster
% project work 1+2
% master biomedical information technology
% university of applied science and arts Dortmund

%% init

clear 
close all
clc

addpath('functions')
addpath('data')

%% connecting to ros

ip = '172.16.9.196';
rosinit(ip, 'NodeName','HostPc') % connectiong to ros master on jetson

%% subcripe to essential topic

zed_sub_odom = rossubscriber('/zedm/zed_node/odom');

%% get data

timeout = 10;
msg_odom = receive(zed_sub_odom,timeout);

pose = odommsg2pose(msg_odom);