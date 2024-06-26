clear;clc;
close all;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
titletext = '  ';
% titletext = 'Full/60m Range/512 Beams/114 Rays';
% titletext = 'Ray Reduced/60m Range/512 Beams/11 Rays';
titletext = 'Ray Range Reduced/10m Range/512 Beams/11 Rays';
clims_base = [-60 -0];
nBeams = 512;
FOV = 90/180*pi();
maxRange = 10;
xPlotRange = 10;
yPlotRange = xPlotRange*cos(45/180*pi());
filename = "/mnt/nfs/home/dgd_rosa/dsor/sonar_data/SonarRawData_000001.csv";
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
topicbw = 29.9e3; % bandwidth
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

Data = csvread(filename,4,0); clearvars Beams dist plotData
plotSkips = 1;
iIndex = 0;
[Beams,dist] = ndgrid(1:length(1:plotSkips:nBeams), (100:length(Data(:,1)))/1500);
for i=2:plotSkips:nBeams+1
    iIndex = iIndex + 1;
    jIndex = 0;
    for j=1:length(Data(:,1))
        jIndex = jIndex + 1;
        plotData(iIndex,jIndex) = Data(j,i)*sqrt(3);
    end
end

sonarBeams = csvread("/mnt/nfs/home/dgd_rosa/dsor/sonar_data/SonarRawData_beam_angles.csv",4)';

range_vector = Data(:,1)';

x = range_vector.*cos(sonarBeams');
y = range_vector.*sin(sonarBeams');

figure;
scatterPointSize = 8;
scatter(x(:),y(:),scatterPointSize,20*log10(abs(plotData(:))),'filled')
clims = clims_base + 20*log10(max(max(abs(plotData))));
caxis(clims)
colorbar
title(titletext)
xlabel('X [m]')
ylabel('Y [m]')
h = colorbar;
ylabel(h,'Echo Level')
axis equal
axis tight
colormap(hot)
set(gca,'Color','k')
xlim(1.02*[0 xPlotRange])
ylim(1.02*[-yPlotRange yPlotRange])
% axis equal;

% caxis([10 65])

figure;
iPlots = 1:30:nBeams;
nPlots = length(1:30:nBeams);
for i=2:nPlots-1
    for j=1:length(Data(:,1))
        temp(j) = Data(j,iPlots(i));
    end
    subplot(1,nPlots-2,i-1);
    plot(abs(temp(1:length(range_vector))),range_vector);
    ylim(1.02*[0 xPlotRange])
end
