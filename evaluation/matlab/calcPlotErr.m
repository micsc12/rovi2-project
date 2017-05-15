cd ~/workspace/rovi2-project/evaluation/matlab

dat = data4;
dat( dat(:,3) == -1,: ) = NaN;
[nrow, ncol] = size(dat);

pos3 = dat(:,1:3);
poscor3 = dat(:,4:6);
velcor3 = dat(:,7:9);
acccor3 = dat(:,10:12);
pospre3 = dat(:,13:15);
velpre3 = dat(:,16:18);
accpre3 = dat(:,19:21);

errcor = zeros(nrow,1);
errpre = zeros(nrow,1);
velcor = zeros(nrow,1);
acccor = zeros(nrow,1);
velpre = zeros(nrow,1);
accpre = zeros(nrow,1);

for i = 1:nrow
    errcor(i) = norm( poscor3(i,:)-pos3(i,:) );
    errpre(i) = norm( pospre3(i,:)-pos3(i,:) );
    velcor(i) = norm( velcor3(i,:) );
    acccor(i) = norm( acccor3(i,:) );
    velpre(i) = norm( velpre3(i,:) );
    accpre(i) = norm( accpre3(i,:) );
end

subplot(3,1,1)
plot(errcor)
legend('Kalman Position Error')
subplot(3,1,2)
plot(velcor, 'g')
legend('Kalman Velocity')
subplot(3,1,3)
plot(acccor, 'r')
legend('Kalman Acceleration')
errcor(isnan(errcor)) = 0;
mean(errcor)
std(errcor)
%plot(eucl3d(dat(:,1:3), [0 0 0]))
%hold on
%plot(errpre, 'r')
%hold off