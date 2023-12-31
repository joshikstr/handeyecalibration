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
addpath('functions/driverThorlabsLTS/')
addpath('data')
addpath('data/images/')

load('cobottapos.mat')

timeoutros = 3; % s 
tagsize = 60;   % mm
targetID = 22;  % ID of used apriltag

%% init Denso Cobotta 

% create caoEngine object
cao = actxserver('CAO.CaoEngine');
% get caoWorkspaces object
ws = cao.Workspaces.Item(int32(0)); 
% create controller object
ctrl_cobotta = ws.AddController('RC8','CaoProv.Denso.RC8','', ...
    'Server = 172.16.6.101');
% create robot controls
cobotta = ctrl_cobotta.AddRobot('arm');

%% init LTS

 % check if LTS class is in the path
 % clone git repo if not 
checkLTSrepo

% list lts devices
SN = LTS.listdevices; 
% create lts object
lts = LTS;
% connect to 1st lts in the list
lts.connect(SN{1})

%% connecting to ros and subcribe to topics

% to start the zed ros wrapper:
% 1. connect via ssh (type in your terminal: ssh 'user'@'ip')
% 2. conform access with the known password
% 3. type:
%   $ cd catkin_ws/
%   $ source devel/setup.bash
%   $ roslaunch zed_wrapper zedm.launch


ip = '172.16.9.196';
try 
    rosinit(ip, 'NodeName','HostPc') 
catch
    rosshutdown
    rosinit(ip, 'NodeName','HostPc')
end

zed_sub_odom = rossubscriber('/zedm/zed_node/odom');
zed_sub_left = rossubscriber('/zedm/zed_node/left/image_rect_gray');
zed_sub_right = rossubscriber('/zedm/zed_node/right/image_rect_gray');
zed_sub_info = rossubscriber('/zedm/zed_node/left/camera_info');

%% get intrinsics and inital odom

caminfo = receive(zed_sub_info,timeoutros);
caminitodommsg = receive(zed_sub_odom,timeoutros);
intrinsics = caminfo2intrinsics(caminfo);
caminitpose = odommsg2pose(caminitodommsg);

%% move lts to inital position

% change homing velocoty
lts.sethomevel(7)
% homing lts
lts.home
% move to initial pos
lts.movetopos(130,30,30) 

%% move cobotta to inital position

% get arm semaphore 
cobotta.Execute('TakeArm');
% start motor
cobotta.Execute('motor',true);
% set robot velocoty
cobotta.Execute('ExtSpeed',40); % 10% 

% move to initial pos
initJoint = 'J(0, 30, 60, 0, 0, 0)';
cobotta.Move(1,initJoint);

%% run sequence

% init
posetagreltocam = [];
istagdetected = [];
posegripper = [];

disp('run sequence...')

for pos = 1:size(cobottapos,1)

    % get joint values pos
    jointvalues = cobottapos(pos,3).jointvaluesstring;
    % move to given pos
    cobotta.Move(1,jointvalues);

    % pause for 800 ms
    pause(.8)

    % get pose gripper rel to base and save in array
    posegripper = [posegripper; curpos2rigidtform3d(cobotta.Execute('CurPos'))];

    % get image zed 
    image = takesnapshot(zed_sub_left,pos);

    % detect AprilTag with target ID in image 
    [posetag, isdetected] = readapriltagtargetID(image,intrinsics,tagsize,targetID);

    % save in array
    posetagreltocam = [posetagreltocam; posetag];
    istagdetected = [istagdetected; isdetected];
end

disp('...done')

% check if enough valid poses detected 
checktagposes(istagdetected)

% merge to table 
cobottapos.posegripper = posegripper;
cobottapos.posetagreltocam = posetagreltocam;
cobottapos.istagdetected = istagdetected;

% delete table rows, where no tag is detected 
cobottavalidpos = cobottapos(cobottapos.istagdetected == true,:);
cobottavalidpos.posID = transpose(1:size(cobottavalidpos,1));

save("data\cobottavalidpos.mat","cobottavalidpos");
