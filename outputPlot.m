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

time=(1:1:length(dataOut(:,1)))*0.001;

figure();
subplot(3,1,1)
plot(time,xCoM);legend('pCoMx','pCoMy','pCoMz');
subplot(3,1,2)
plot(time,vCoM);legend('vCoMx','vCoMy','vCoMz');
subplot(3,1,3)
plot(time,eul);legend('roll','pitch','yaw');

figure();
subplot(2,1,1)
plot(time,peW(:,1:3));legend('pewrX','pewrY','pewrZ');
subplot(2,1,2)
plot(time,peW(:,4:6));legend('pewrX','pewrY','pewrZ');


figure();
subplot(2,2,1)
plot(time,dataOut(:,1:3));legend('rfx','rfy','rfz');
subplot(2,2,2)
plot(time,dataOut(:,4));legend('rtauz');
subplot(2,2,3)
plot(time,dataOut(:,5:7));legend('lfx','lfy','lfz');
subplot(2,2,4)
plot(time,dataOut(:,8));legend('ltauz');

figure();
subplot(2,1,1)
plot(time,legInd);
subplot(2,1,2)
plot(time,dataOut(:,3));
hold on;
plot(time,dataOut(:,7))


