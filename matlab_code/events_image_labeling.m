%% Load data
% clear

% load from rosbag
filename = '/Users/huangxiaoqian/Documents/Research/rpg_event_lifetime-master/matlab/data/multi-objects-practical.bag';
% dataset = 'multi-objects-practical';
% filename = '/Users/huangxiaoqian/Documents/Research/rpg_event_lifetime-master/matlab/data/multi-objects-practical.bag'

bag=rosbag(filename)
bag.AvailableTopics

dvs_events_topic=select(bag, 'Topic', '/dvs/events'); 
dvs_events = readMessages(dvs_events_topic,'DataFormat','struct'); % topic & msg of DAVIS

n=0;
t0=0;
events=[];
len=length(dvs_events)
% t=[];
for i=1:length(dvs_events)
%     Time(i)=(i-1)*FrameMs; % time interval=10ms
    % reading events and speed
    for j=1:length(dvs_events{i}.Events)
        n=n+1;
        x(n)=double(dvs_events{i}.Events(j).X);
        y(n)=double(dvs_events{i}.Events(j).Y); 
        p(n)=double(dvs_events{i}.Events(j).Polarity);  
        if p(n) == 0
            p(n) = -1;
        end
        
        % representation of time    
        format long
        timeo = rostime(dvs_events{i}.Events(j).Ts.Sec,dvs_events{i}.Events(j).Ts.Nsec);
        if i==1 & j==1
           t0 = double(seconds(timeo));
        end
        t(n) = floor((double(seconds(timeo))-t0)*1E6); 
        
        event = [x(n), y(n), p(n), t(n)];
        events = [events; event];
        
   
    
    end   
end
%% get image
y=events(:,1); %0-345
x=events(:,2); %0-259
p=events(:,3); %0-259
t=events(:,4);

% for 1st 10ms, i =207.
idx1=2000;

image=zeros(260,346,3); % l1: mask, l2: polarity, l3: time temporal
time=zeros(260,346,1); 
% re=zeros(260,346,1);

% 
for i=1:idx1
    xi=x(i)+1;yi=y(i)+1;
    image(xi,yi,1)=1;
%     time(xi,yi)=t(i);
    image(xi,yi,2)=p(i);
    image(xi,yi,3)=t(i);
%     re(xi,yi) = re(xi,yi)+1;
end
    
% coeff=re/max(max(re));
% I=image.*coeff;
% imshow(I)

figure
imshow(image)
% imshow(time)
figure
image1=bwareaopen(image(:,:,1),2);
imshow(image1)
noise=image(:,:,1)-image1;
noise=noise*(-1);

[r_row,r_col,r_v] = find(image1>0);

%%
X=events(1:idx1,1:2);
sigma = 0.1314;
nbclusters = 2;
[clusters, evalues, evectors] = spcl(data, nbclusters, sigma, 'kmean', [2 2]);
%% k-means data--2 cluster
% https://www.mathworks.com/help/stats/kmeans.html
% https://towardsdatascience.com/the-5-clustering-algorithms-data-scientists-need-to-know-a36d136ef68


% X=events(1:idx1,1:2);
X=cat(2,r_col,r_row);
k=2

opts = statset('Display','final');
[idx,C] = kmeans(X,k,'Distance','cityblock',...
    'Replicates',5,'Options',opts);
% [idx,C,sumd,d,midx,info] = kmedoids(X,2,'Distance','cityblock','Options',opts);

% sort center points, give ordered cluster ID
Co=C
C=sort(C)
n=2;
ind(1)=1,ind(2)=2;
if C~=Co
    for i=1:n
        ind(i)=find(Co(:,1)==C(i,1));
%         b=find(Co(:,1)==C(2,1))
    end
end

obj1=cat(2,X(idx==ind(1),2),X(idx==ind(1),1));
obj2=cat(2,X(idx==ind(2),2),X(idx==ind(2),1));
obj1_img = zeros(260,346,1); 
obj2_img = zeros(260,346,1); 
for i=1:length(obj1)
    obj1_img(obj1(i,1),obj1(i,2))=1;
end
for i=1:length(obj2)
    obj2_img(obj2(i,1),obj2(i,2))=2;
end

mask_label = noise+obj1_img+obj2_img;
image(:,:,1)= mask_label;

data=[];
for i=1:260
    for j=1:346
        val=image(i,j,1);
        if val~=0
            xy=[i,j];
            label=val;
            pol=image(i,j,2);
            tt=image(i,j,3);
            arr=[xy,label,pol,tt];
            data=[data;arr];
        end
    end
end

data=sortrows(data,5);


figure;
plot(X(idx==ind(1),1),X(idx==ind(1),2),'r.','MarkerSize',12)
hold on
plot(X(idx==ind(2),1),X(idx==ind(2),2),'b.','MarkerSize',12)
% hold on
% plot(X(idx==3,1),X(idx==3,2),'g.','MarkerSize',12)
plot(C(:,1),C(:,2),'kx',...
     'MarkerSize',15,'LineWidth',3) 
legend('Cluster 1','Cluster 2','Centroids',...
       'Location','NW')
title 'Cluster Assignments and Centroids'
hold off

%% k-means data--3 cluster
% https://www.mathworks.com/help/stats/kmeans.html
% https://towardsdatascience.com/the-5-clustering-algorithms-data-scientists-need-to-know-a36d136ef68


% X=events(1:idx1,1:2);
X=cat(2,r_col,r_row);
k=3

opts = statset('Display','final');
[idx,C] = kmeans(X,k,'Distance','cityblock',...
    'Replicates',5,'Options',opts);
% [idx,C,sumd,d,midx,info] = kmedoids(X,2,'Distance','cityblock','Options',opts);

% sort center points, give ordered cluster ID
Co=C
C=sort(C)
n=3;
ind(1)=1,ind(2)=2,ind(3)=3;
if sum(abs(sum(C'-Co')))>0 %C~=Co
    sprintf('C!=Co');
    for i=1:n
        ind(i)=find(Co(:,1)==C(i,1));
%         b=find(Co(:,1)==C(2,1))
    end
end

obj1=cat(2,X(idx==ind(1),2),X(idx==ind(1),1));
obj2=cat(2,X(idx==ind(2),2),X(idx==ind(2),1));
obj3=cat(2,X(idx==ind(3),2),X(idx==ind(3),1));
obj1_img = zeros(260,346,1); 
obj2_img = zeros(260,346,1); 
obj3_img = zeros(260,346,1); 
for i=1:length(obj1)
    obj1_img(obj1(i,1),obj1(i,2))=1;
end
for i=1:length(obj2)
    obj2_img(obj2(i,1),obj2(i,2))=2;
end
for i=1:length(obj3)
    obj3_img(obj3(i,1),obj3(i,2))=3;
end

mask_label = noise+obj1_img+obj2_img+obj3_img;
image(:,:,1)= mask_label;

data=[];
for i=1:260
    for j=1:346
        val=image(i,j,1);
        if val~=0
            xy=[i,j];
            label=val;
            pol=image(i,j,2);
            tt=image(i,j,3);
            arr=[xy,label,pol,tt];
            data=[data;arr];
        end
    end
end

datas=sortrows(data,5);


figure;
plot(X(idx==ind(1),1),X(idx==ind(1),2),'r.','MarkerSize',12)
hold on
plot(X(idx==ind(2),1),X(idx==ind(2),2),'b.','MarkerSize',12)
hold on
plot(X(idx==ind(3),1),X(idx==ind(3),2),'g.','MarkerSize',12)
plot(Co(:,1),Co(:,2),'kx',...
     'MarkerSize',15,'LineWidth',3) 
legend('Cluster 1','Cluster 2','Cluster 3','Centroids',...
       'Location','NW')
title 'Cluster Assignments and Centroids'
hold off

%% https://www.mathworks.com/help/stats/dbscan.html

% Parameters for data generation
% N = 300;  % Size of each cluster
% r1 = 0.5; % Radius of first circle
% r2 = 5;   % Radius of second circle
% theta = linspace(0,2*pi,N)';
% 
% idx = dbscan(double(X),1,5);
% gscatter(X(:,1),X(:,2),idx);
% title('DBSCAN Using Euclidean Distance Metric')

n = 20;
X = int8(X);
[idx1,V,D] = spectralcluster(X,2)
gscatter(X(:,1),X(:,2),idx1);

%% GMM
GMModel = fitgmdist(X,2);
figure
y = [zeros(1000,1);ones(1000,1)];
h = gscatter(X(:,1),X(:,2),y);
hold on
gmPDF = @(x,y) arrayfun(@(x0,y0) pdf(GMModel,[x0 y0]),x,y);
g = gca;
fcontour(gmPDF,[g.XLim g.YLim])
title('{\bf Scatter Plot and Fitted Gaussian Mixture Contours}')
legend(h,'Model 0','Model1')
hold off
%% soft GMM not good
gm = fitgmdist(X,2);
threshold = [0.37 0.63];
P = posterior(gm,X);
n = size(X,1);
[~,order] = sort(P(:,1));

% figure
% plot(1:n,P(order,1),'r-',1:n,P(order,2),'b-')
% legend({'Cluster 1', 'Cluster 2'})
% ylabel('Cluster Membership Score')
% xlabel('Point Ranking')
% title('GMM with Full Unshared Covariances')

idx = cluster(gm,X);
idxBoth = find(P(:,1)>=threshold(1) & P(:,1)<=threshold(2)); 
numInBoth = numel(idxBoth)

figure
gscatter(X(:,1),X(:,2),idx,'rb','+o',5)
hold on
plot(X(idxBoth,1),X(idxBoth,2),'ko','MarkerSize',10)
legend({'Cluster 1','Cluster 2','Both Clusters'},'Location','SouthEast')
title('Scatter Plot - GMM with Full Unshared Covariances')
hold off
