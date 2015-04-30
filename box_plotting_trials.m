% Plot the time trials
close all; clc;
load('timeTrials.mat');

figure(2);
hold on
% val =  reshape(timeTrials,120,1);
% Nrobots_h = [Nrobots;Nrobots;Nrobots;Nrobots;Nrobots;Nrobots;Nrobots;Nrobots;Nrobots;Nrobots];
% bin =  reshape(Nrobots_h,120,1);
% boxplot(val,bin);

a = 1/300000; %quadratic factor
b = 1/100000000; %cubic factor
medians = median(timeTrials);
plot(Nrobots,medians,'+b','MarkerSize',10);
plot(Nrobots,a*Nrobots.^2,':','color',[0.9 0.6 0]);
plot(Nrobots,b*Nrobots.^3,'-.','color',[0.5 0 0.5]);
l = legend('Hungarian','Quadratic','Cubic');
set(l,'Location', 'NorthWest');

maxes = max(timeTrials);
mins = min(timeTrials);
plot(Nrobots,maxes,'.r','MarkerSize',5,'Marker','^');
plot(Nrobots,mins,'.r','MarkerSize',5,'Marker','v');
plot(Nrobots,medians,'+b','MarkerSize',10);

title('Robots vs Computation')
xlabel('Number of Robots, N')
ylabel('Time to Compute (s)')
ax = gca;
set(ax,'YScale','log','XScale','log', 'MinorGridLineStyle', ':', 'GridLineStyle', '-', 'Color', [1 1 1],'LineWidth', 0.1);
axis([10^0*5, 10^3*2, 10^-5, 10^2]);
grid off



