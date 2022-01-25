% This script transforms the originally recorded MNIST-DVS database into a replica with the following changes:
%
% a) If "Remove_75hz" is set to '1' it will reshuffle timestamps to eliminate the 75hz LCD screen refresh rate harmonic
%    in the timestamp sequence. Events order is preserved, but timesptamps are recomputed randomly. Randomness can be adjusted
%    through parameter "Nosie_factor". According to our experience, setting it to '2' results in a similar noise floor
%    than the one orignally available.
%
% b) The MNIST-DVS digits move along the screen. By setting "Stabilize = 1" the moving trajectory of the center is subtracted
%    from the (x,y) event coordinates. This way, digits appear stabilized in the center (although not perfectly).
%
% c) MNIST-DVS events include a polarity bit which indicates whether light increased or decreased at that pixel location.
%    Many times, specially when doing object recognition, this polarity bit is not used. This polarity bit can be set
%    constant by setting parameter "Remove_Polarity = 1". This way, when displaying the resulting processed MNIST-DVS digits
%    on jAER, all events will be displayed with the same polarity.
%
% 
% This script should be run from the directory where the originally recorded "grabbed_dataxxx" folders are stored,
% and will generate a set of parallel folders named "processed_dataxxx".
%
% Script written by Bernabe Linares-Barranco, in Oct-2015. For questions and comments please contact bernabe(at)imse-cnm.csic.es
%
%



clear

Noise_factor=2;
Remove_75hz=1;
Stabilize = 1;
Remove_Polarity = 1;
TT = 0.2982
dd2=[];

    scale=[4 8 16];
    addpath(cd);
    nsamp = 1000;
    if nsamp>1000
        error('nsamp must be <= 1000');
    end
    
    for dig=0:9
        name_dir1 = sprintf('processed_data%d',dig);
        if exist(name_dir1,'dir') ~= 0
            error('Directory %s already exists',name_dir1);
        else
           %mkdir(name_dir1);
        end
        for isc=1:3
            sc=scale(isc);
            name_dir2 = sprintf('processed_data%d/scale%d',dig,sc);
            if exist(name_dir2,'dir') ~= 0
                error('Directory %s already exists',name_dir2);
            else
                mkdir(name_dir2)
            end
            
            for i=1:nsamp
                fname_in=sprintf('grabbed_data%d/scale%d/mnist_%d_scale%02d_%04d.aedat',dig,sc,dig,sc,i);
                fname_out=sprintf('processed_data%d/scale%d/mnist_%d_scale%02d_%04d.aedat',dig,sc,dig,sc,i);
                dd=dat2mat(fname_in);
                
                if Remove_75hz == 1
                    tend=dd(end,1);
                    ii=dd(:,1);
                    ii_max=max(ii);
                    dii=diff(ii);
                    mean_dii=mean(dii);
                    std_dii=std(dii);
                    ii2=cumsum(mean_dii+Noise_factor*std_dii*randn(length(ii),1));
                    ii2=ii2-min(ii2);
                    ii2_max=max(ii2);
                    ii2=round(ii2*ii_max/ii2_max);
                    ii2=sort(ii2);
                    dd(:,1)=ii2;
                end
                
                if Stabilize ==1
                    Trampy=TT*1e6;
                    Trampx=2*TT*1e6;
                    DY = 5;
                    DX=10;
                    Ymin = 58;
                    Xmin=58;
                    sy=0.5;
                    sx=0.5;
                    yc0 = abs(mod(dd(:,1),2*Trampy)-Trampy)/Trampy*DY + Ymin;
                    yc0 = yc0 + sy*randn(length(yc0),1);
                    yc=127-round(yc0);
                    xc0 = abs(mod(dd(:,1)+1.5*Trampx,2*Trampx)-Trampx)/Trampx*DX + Xmin;
                    xc0 = xc0 + sx*randn(length(xc0),1);
                    xc=127-round(xc0);
                    dd(:,4)=dd(:,4)-xc+63;
                    dd(:,5)=dd(:,5)-yc+63;
                    ind=find(dd(:,4)>=0 & dd(:,4)<128 & dd(:,5)>=0 & dd(:,5)<128);
                    dd=dd(ind,:);
                end
                
                if Remove_Polarity == 1
                    dd(:,6)=1;
                end
                
                mat2dat(dd,fname_out);
                 
            end
        end
    end