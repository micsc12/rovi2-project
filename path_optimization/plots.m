data =[ 15, 12.4347, 3, 4.01655, 5, 4.01655; 
13, 10.7271, 2, 2.62241, 2, 2.62241; 
12, 10.1658, 3, 4.88995, 4, 4.26068; 
13, 11.675, 4, 6.91173, 6, 6.62948; 
11, 8.66019, 3, 3.5671, 4, 3.28944; 
12, 10.6157, 3, 2.57617, 5, 2.57617; 
10, 9.17808, 3, 4.22565, 5, 3.93924; 
13, 11.2498, 3, 3.59142, 4, 2.76146; 
15, 13.3232, 4, 7.30981, 5, 6.78589; 
15, 13.4909, 5, 7.08375, 4, 5.3614; 
16, 15.1778, 3, 4.46591, 5, 4.16063; 
16, 15.1778, 3, 4.46591, 5, 4.16063; 
13, 11.5952, 3, 4.48661, 5, 3.93746; 
15, 12.435, 3, 4.01609, 5, 4.01609; 
15, 12.435, 3, 4.01609, 5, 2.94748; 
15, 12.435, 3, 4.01609, 6, 3.28493; 
13, 11.1112, 5, 8.01652, 9, 7.64682; 
15, 12.435, 3, 4.01609, 5, 4.01609; 
13, 11.1112, 5, 8.01652, 4, 6.17994; 
8, 6.60057, 3, 4.88995, 4, 4.39427; 
10, 9.39747, 3, 4.80648, 4, 3.6153; 
14, 13.1609, 3, 4.38209, 5, 4.03408; 
10, 8.08959, 5, 6.86923, 5, 5.50696; 
16, 13.7284, 3, 3.6281, 4, 2.75315; 
12, 11.0558, 5, 7.05685, 5, 5.60696; 
11, 8.82959, 3, 4.05616, 6, 3.0035; 
12, 10.3766, 5, 7.13313, 16, 6.98258; 
12, 10.608, 4, 7.662, 4, 5.86988; 
12, 10.2025, 3, 4.60653, 5, 3.19131; 
10, 8.87877, 3, 3.74203, 4, 2.89169]

hold off;
data2 = [data(:,2) data(:,4) data(:,6)] 
[H,AX] = plotmatrix(data2);
%plotmatrix(data2)
ylabel(AX(1,1),'Initial length')
ylabel(AX(2,1),'After pruning')
ylabel(AX(3,1),'After shortcut')
xlabel(AX(3,1),'Initial length')
xlabel(AX(3,2),'After pruning')
xlabel(AX(3,3),'After shortcut')
%%

mean(data2)
boxplot(data2,'Labels',{'Initial length','After pruning','After pruning + shortcut'})
title('Path lengths before and after optimization')
ylabel('Length in Q-space');

axis([0.5 3.5 0 16])



%% d
edges = [0:0.5:15];
hold off
histogram(data(:,2),edges,'FaceColor','blue','Normalization','pdf')
xlabel('Distance in Q-space')
hold on;
histogram(data(:,4),edges,'FaceColor','green','Normalization','pdf')
xlabel('Distance in Q-space')

histogram(data(:,4),edges,'FaceColor','green','Normalization','pdf')
xlabel('Distance in Q-space')



% get mean and variance
sample_mean = mean(data)
sample_var  = var(data)
std_var = sqrt(sample_var)
y = 0:0.1:15;
f = exp(-(y-sample_mean(2)).^2./(2*std_var(2)^2))./(std_var(2)    *sqrt(2*pi));
plot(y,f,'LineWidth',1.5)

f = exp(-(y-sample_mean(4)).^2./(2*std_var(4)^2))./(std_var(4)    *sqrt(2*pi));
plot(y,f,'LineWidth',1.5)

