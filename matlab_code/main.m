%% Event Lifetime Estimation
% E. Mueggler, C. Forster, N. Baumli, G. Gallego, D. Scaramuzza
% "Lifetime Estimation of Events from Dynamic Vision Sensors"
% In: IEEE International Conference on Robotics and Automation (ICRA), 
% Seattle, 2015.
% PDF: http://rpg.ifi.uzh.ch/docs/ICRA15_Mueggler.pdf

%% Load data
% clear
data_folder = 'data';

% Experiment 1: stripes dataset
%dataset = 'stripes.mat';

% Experiment 2: Garfield
% dataset = 'garfield.mat';

% Experiment 3: quadrotor flip
% dataset = 'flip.mat';

% Experiment 4: building
%dataset = 'building.mat';

% load data
% events_o = loadEvents([data_folder, '/', dataset]);

% load from rosbag
filename = '/Users/huangxiaoqian/Documents/Research/rpg_event_lifetime-master/matlab/data/3.bag';
% dataset = 'multi-objects-practical';
% filename = '/Users/huangxiaoqian/Documents/Research/rpg_event_lifetime-master/matlab/data/multi-objects-practical.bag'
[events, v, pose, t] = loadEvents_rosbag(filename);

%% Parameters
% Window Size
N = 9; % default-5
% Estimated fraction of outliers for RANSAC algorithm
epsilon = 0.4; % default-0.4
% Euclidian distance threshold for RANSAC algorithm
mu = 0.001; % default-0.0001
% Regularization
reg = true;
% Visualization during computation
vis = false;%true;%
% Show velocity on visualization
show_vel = true;%

%% Compute lifetime
% events_with_lifetime = [vx, vy, t, te(time prediction error)]
 events_with_lifetime = calcVelocity(events, N, epsilon, mu, reg, vis, v, pose); 
%events_with_lifetime_p = calcVelocity(events(10000:26000,:), N, epsilon, mu, reg, vis, v, pose);
% events_with_lifetime_p_d = events_with_lifetime_p(:,9)+ abs(min(events_with_lifetime_p(:,9)));
%% radialDistortion calibration
IntrinsicMatrix = [371.33899766 0 0; 0 371.88405087 0; 159.89008556 122.45995657 1];
radialDistortion = [-3.63353987e-01  4.35559090e-01 -9.16729641e-01]; 
tangentialDistortion = [7.33222111e-04 -2.59464488e-04];
cameraParams = cameraParameters('IntrinsicMatrix',IntrinsicMatrix,'RadialDistortion',radialDistortion); 

Im=zeros(346,260,3);
% Im=zeros(4,50000);
for i=1:1000
    xi=events(i,1);
    yi=events(i,2);
    Im(xi,yi,1)=events(i,3);
    Im(xi,yi,2)=events(i,4);
    Im(xi,yi,3)=Im(xi,yi,3)+1;
end
   
I = Im;
J = undistortImage(I,cameraParams);
figure
imshow(I);
figure
imshow(J);

diff=I-J;
figure
imshow(diff);

%% robotic pose to camera pose 
pos = ones(4,4);
pos(1,4)=pose(1,1);
pos(2,4)=pose(1,2);
pos(3,4)=pose(1,3);

TCP_to_cam = [ 0.99997541  0.00109529 -0.00692689 -0.00131305;
 0.00692358  0.0030003   0.99997153  0.04473792;
 0.00111604 -0.9999949   0.00299264  0.05418831;
 0.          0.          0.          1.        ];

pos
pose_cam = pos*TCP_to_cam

vel=ones(4,4);
vel(1,4)=v(1,1);
vel(2,4)=v(1,2);
vel(3,4)=v(1,3);

vel
vel_cam = vel*TCP_to_cam



%% depth processing
fx = 371.33899766; fy = 371.88405087;
f = 371;%(fx + fy)/2*1e-5;
% vel=v;
vx=vel(:,1);%m/s
vy=vel(:,2);
vz=vel(:,3);
u=events_with_lifetime(:, 1);%*18.5*1e-6; % pixels
v=events_with_lifetime(:, 2);%*18.5*1e-6; % 
du = events_with_lifetime(:, 5)*1e5; %*18.5;%m/s, um/us
dv = events_with_lifetime(:, 6)*1e5; %*18.5;%m/s
zu=(-f*vx+u.*vz)./du;
zv=(-f*vy+v.*vz)./dv;
zu_s=-f*vx./du;
zv_s=-f*vy./dv;
zuv=[zu, zv];
zuv_s=[zu_s, zv_s];
zuv_s_c=sqrt(zu_s.^2 + zv_s.^2);
for i =1:length(zuv)
    zuv1 = zuv(i,1);
    if  zuv1(isinf(zuv1))
        zuv(i,1)=0;
    end
    zuv2 = zuv(i,2);
    if  zuv1(isinf(zuv2))
        zuv(i,2)=0;
    end
end

zuvi=1./zuv;
% v_events = sqrt(events_with_lifetime(:,5).^2+events_with_lifetime(:,6).^2);
% v_cam = sqrt(v(:,1).^2+v(:,2).^2);
% depth_events = pose(:,3).*(v_cam./(v_events));
% 
% depth = depth_events(~isinf(depth_events));
% depth = (depth-min(depth))./(max(depth)-min(depth));


% depth = depth(depth~=1)*10;
% depth_events = pose(:,3).*(v./(v_events+v));

% depth_events = events_with_lifetime(:,9);
%% Create video
% visualization parameter for lifetime
% if strcmp(dataset, 'flip.mat')
% % if strcmp(dataset, events)
% %     cmax = 500;
% % else
% %     cmax = 14000;
% % end

cmax=500;
dataset='practical.bag';
% video using lifetime
% dispOutput(events_with_lifetime, show_vel, -1, [dataset, '_lifetime'], cmax);
% % video using depth
dispDepth_new(events_with_lifetime(:,:), events_with_lifetime(:,9), show_vel, -1, [dataset, '_dt30'], cmax);%D_norm, depth_events
% dispDepth_new(events_with_lifetime_p, events_with_lifetime_p(:,9), show_vel, -1, [dataset, '_dt30'], cmax);%D_norm, depth_events

% video using fixed time interval of 30ms
% dispDepth(events_with_lifetime, show_vel, 30, [dataset, '_dt30'], cmax);

% visualization parameter for depth

%% test depth 
d=mean(pose(:,3));
depth = events_with_lifetime_p(:,9);
depth = depth(~isinf(depth));
% depth = depth(depth<d);
% depth = depth(depth>0);
% depth = depth(depth>0);
% depth = abs(depth);
% depth = depth(depth<19);
% depth = (depth-min(depth))./(max(depth)-min(depth));

%% test depth
d(1)=0;
for i=2:length(pose)
    if abs(v(i,1)-v(i-1,1))>0
        d(i)= (pose(i,1)-pose(i-1,1))/(v(i,1)-v(i-1,1));
    else
            d(i)=d(i-1);           
    end
end

d=d';

%% test events_depth
dvx(1)=0;
dx(1)=0;
d(1)=0;
D=[];
for i=2:length(events_with_lifetime)
%     if events_with_lifetime(i,5)==0
%         events_with_lifetime(i,5)=events_with_lifetime(i-1,5);
%     end
    dx(i)=(events_with_lifetime(i,4)-events_with_lifetime(i-1,4))*events_with_lifetime(i,5);
    dvx(i)=(events_with_lifetime(i,5)-events_with_lifetime(i-1,5));
    if dvx(i)~=0
        d(i)=dx(i)/dvx(i);
%         if d(i)>80
%             d(i)=d(i-1);
%         end
    else
        d(i)=0;%d(i-1);
    end
    D=[D; d(i)];
%     D=[D;abs(d(i))];
end

D_norm = (D-min(D))./(max(D)-min(D));

% D_norm = (D-min(D(D>0.01)))./(max(D(D<100))-min(D(D>1)));
% D_norm = abs(D_norm);

%% 
t= [1:10796];
figure, hold on 
plot(t, v(:,1));
plot(t, v(:,2));
hold off
plot
