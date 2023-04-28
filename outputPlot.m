clear variables;
close all;
dataOut=load('OutputData.txt');
stateData=load('StateData.txt');

xCoM=stateData(:,1:3);
vCoM=stateData(:,4:6);
peW=stateData(:,7:12);
legInd=stateData(:,13:14);
legIndPha=stateData(:,15:16);
eul=stateData(:,17:19);
omegaW=stateData(:,20:22);

last_nWSR=dataOut(:,11);
last_cpuTime=dataOut(:,12);
tauR=dataOut(:,13:17);
tauL=dataOut(:,18:22);
IcmdR=dataOut(:,23:27);
IcmdL=dataOut(:,28:32);
pCoMOff=dataOut(:,33:35);

time=(1:1:length(dataOut(:,1)))*0.001;

figure();
subplot(3,1,1)
plot(time,xCoM);legend('pCoMx','pCoMy','pCoMz');
ylabel('position (m)');
subplot(3,1,2)
plot(time,vCoM);legend('vCoMx','vCoMy','vCoMz');
ylabel('velocity (m/s)');
subplot(3,1,3)
plot(time,eul);legend('roll','pitch','yaw');
ylabel('attitude (rad)')

figure();
subplot(2,1,1)
plot(time,peW(:,1:3));legend('pewrX','pewrY','pewrZ');
subplot(2,1,2)
plot(time,peW(:,4:6));legend('pewrX','pewrY','pewrZ');


figure();
subplot(2,2,1)
plot(time,dataOut(:,1:3));legend('rfx','rfy','rfz');
ylabel('force (N)');xlabel('time (s)')
subplot(2,2,2)
plot(time,dataOut(:,4));legend('rtauz');
ylabel('torque (Nm)');xlabel('time (s)')
subplot(2,2,3)
plot(time,dataOut(:,5:7));legend('lfx','lfy','lfz');
ylabel('force (N)');xlabel('time (s)')
subplot(2,2,4)
plot(time,dataOut(:,8));legend('ltauz');
ylabel('torque (Nm)');xlabel('time (s)')

figure();
subplot(2,1,1)
plot(time,legInd);
subplot(2,1,2)
plot(time,dataOut(:,3));
hold on;
plot(time,dataOut(:,7))

figure();
subplot(2,1,1)
plot(time,last_nWSR);
ylabel('nWSR');
subplot(2,1,2)
plot(time,last_cpuTime);
ylabel('cpuTime');

figure();
subplot(2,1,1)
plot(time,tauR);
legend('m1','m2','m3','m4','m5')
ylabel('Rleg torque')
subplot(2,1,2)
plot(time,tauL);
legend('m1','m2','m3','m4','m5')
ylabel('Lleg torque')


figure();
subplot(2,1,1)
plot(time,IcmdR);
legend('m1','m2','m3','m4','m5')
ylabel('Rleg Icmd')
subplot(2,1,2)
plot(time,IcmdL);
legend('m1','m2','m3','m4','m5')
ylabel('Lleg Icmd')











