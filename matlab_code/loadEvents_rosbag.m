function [events, vt, pose, t] =loadEvents_rosbag(fileName)

%fileName = '/Users/huangxiaoqian/Documents/Research/rpg_event_lifetime-master/matlab/data/output.bag'
bag=rosbag(fileName)
bag.AvailableTopics

start_time = 1.634654475e9;
end_time = start_time + 2;
time_range = [start_time, end_time];

dvs_events_topic=select(bag, 'Topic', '/dvs/events', 'Time', time_range); 
dvs_events = readMessages(dvs_events_topic,'DataFormat','struct'); % topic & msg of DAVIS
% dvs_events_ts = timeseries(dvs_events_topic);

dvs_pose_topic = select(bag, 'Topic', '/dvs/pose', 'Time', time_range); 
dvs_pose = readMessages(dvs_pose_topic,'DataFormat','struct'); % topic & msg of DAVIS_pose
dvs_pol_ts = timeseries(dvs_pose_topic).Time;

dvs_vel_topic = select(bag, 'Topic', '/dvs/vel', 'Time', time_range); 
dvs_vel = readMessages(dvs_vel_topic,'DataFormat','struct'); % topic & msg of DAVIS_velocity_linear & angular
dvs_vel_ts = timeseries(dvs_vel_topic).Time;

dvs_spd_topic = select(bag, 'Topic', '/dvs/spd', 'Time', time_range); 
dvs_spd = readMessages(dvs_spd_topic,'DataFormat','struct'); % topic & msg of DAVIS_speed: overall speed

n=0;
t0=0;
events=[];
pose=[];
v=[];
% t=[];
for i=1:length(dvs_events)
%     Time(i)=(i-1)*FrameMs; % time interval=10ms
       i
    % reading events and speed
    for j=1:length(dvs_events{i}.Events)
        n=n+1;
        x(n)=double(dvs_events{i}.Events(j).X);
        y(n)=double(dvs_events{i}.Events(j).Y); 
        p(n)=double(dvs_events{i}.Events(j).Polarity);  
%         t = rostime(dvs_events{i}.Header.Stamp.Sec,dvs_events{i}.Header.Stamp.Nsec);
%         ts = t.Sec + double(t.Nsec)/double(1E9); 
           
        
        if p(n) == 0
            p(n) = -1;
        end
        
        % representation of time    
        format long
        timeo = rostime(dvs_events{i}.Events(j).Ts.Sec,dvs_events{i}.Events(j).Ts.Nsec);
        if i==1 & j==1
           t0 = seconds(timeo);
        end
        ts = seconds(timeo);
%         t(n) = floor((double(seconds(timeo))-t0)*1E6); 
        t(n) = ts;
        absv=abs(dvs_vel_ts-ts);
        [a,ind] = find(min(absv')==absv');
        
        event = [x(n), y(n), p(n), t(n)];
        events = [events; event];
        
        % dvs moving speed
%         v(n)=double(dvs_spd{i}.Data); 
%         v(n)=double(sqrt(dvs_vel{i}.Linear.X^2 + dvs_vel{i}.Linear.Y^2)); 
        ind=ind(1)
        v_x(n)=dvs_vel{ind}.Linear.X;
        v_y(n)=dvs_vel{ind}.Linear.Y;
        v_z(n)=dvs_vel{ind}.Linear.Z;
        v_r1(n)=dvs_vel{ind}.Angular.X;
        v_r2(n)=dvs_vel{ind}.Angular.Y;
        v_r3(n)=dvs_vel{ind}.Angular.Z;
        v_xyz = [v_x(n), v_y(n), v_z(n), v_r1(n), v_r2(n), v_r3(n)];
        v=[v;v_xyz];
        
        % dvs pose
%         [b,ind] = find(abs(dvs_pol_ts-double(ts)));
%         indp = ind(1);
        pose_x(n) = dvs_pose{ind,1}.Pose.Position.X;
        pose_y(n) = dvs_pose{ind,1}.Pose.Position.Y;
        pose_z(n) = dvs_pose{ind,1}.Pose.Position.Z;
        pose_xyz = [pose_x(n), pose_y(n), pose_z(n)];
        pose = [pose; pose_xyz];
        
        
        if i==60
            a=0;
        end
    end   
end
vt = v;

% for i=1:length(dvs_spd)
%     v_raw(i)=dvs_spd{i}.Data; 
% end

end


