clear variables;
close all;
dataOut=load('OutputData.txt');
stateData=load('stateData_QP.txt');

xCoM=stateData(:,1:3);
vCoM=stateData(:,4:6);
peW=stateData(:,7:12);
legInd=stateData(:,13:14);
legIndPha=stateData(:,15:16);
eul=stateData(:,17:19);
omegaL=stateData(:,20:22);
yaw0=stateData(:,37);
eul(:,3)=eul(:,3)-yaw0;
pe_L_fk=stateData(:,38:43);
pe_Body_Accumu_OriRec=stateData(:,44:49);
ufe_Ori=stateData(:,50:57);
wL_filtered=stateData(:,58:60);
eul_filtered=stateData(:,61:63);
ddx_d_ori=stateData(:,64:66);
ddw_d_ori=stateData(:,67:69);
qp_cpu_time_Ori=stateData(:,70);
qp_nWSR_Ori=stateData(:,71);


ufe_Now=dataOut(:,1:8);
last_nWSR=dataOut(:,11);
last_cpuTime=dataOut(:,12);
tauR=dataOut(:,13:17);
tauL=dataOut(:,18:22);
IcmdR=dataOut(:,23:27);
IcmdL=dataOut(:,28:32);
pCoMOff=dataOut(:,33:35);
ddx_d_cmd=dataOut(:,36:38);
ddx_d_qpRes=dataOut(:,39:41);
ddw_d_cmd=dataOut(:,42:44);
ddw_d_qpRes=dataOut(:,45:47);
pe_Body_Old=dataOut(:,48:53);
pe_Body_delta=dataOut(:,54:59);
pe_Body_Accumu=dataOut(:,60:65);


time=(1:1:length(dataOut(:,1)))*0.001;

figure();
subplot(4,1,1)
plot(time,xCoM);legend('pCoMx','pCoMy','pCoMz');
ylabel('position (m)');
subplot(4,1,2)
plot(time,vCoM);legend('vCoMx','vCoMy','vCoMz');
ylabel('velocity (m/s)');
subplot(4,1,3)
plot(time,eul);
hold on;
plot(time,eul_filtered);
legend('roll','pitch','yaw','roll_LP','pitch_LP','yaw_LP');
ylabel('attitude (rad)')
subplot(4,1,4)
plot(time,omegaL);
hold on;
plot(time,wL_filtered);
legend('wx','wy','wz','wx_LP','wy_LP','wz_LP');
ylabel('omegaL (rad/s)')


figure();
subplot(2,1,1)
plot(time,peW(:,1:3));legend('pewrX','pewrY','pewrZ');
subplot(2,1,2)
plot(time,peW(:,4:6));legend('pewrX','pewrY','pewrZ');

figure();
subplot(2,1,1)
plot(time,pe_L_fk(:,1:3));legend('peL-fb-rX','peL-fb-rY','peL-fb-rZ');
subplot(2,1,2)
plot(time,pe_L_fk(:,4:6));legend('peL-fb-lX','peL-fb-lY','peL-fb-lZ');

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

figure("Name",'ddx-ddw');
for i=1:1:3
subplot(2,3,i)
plot(time,ddx_d_cmd(:,i));
hold on;
plot(time,ddx_d_ori(:,i));
plot(time,ddx_d_qpRes(:,i));
legend('ddx-d-cmd','ddx-d-ori','ddx-d-qp')
end
for i=1:1:3
subplot(2,3,i+3)
plot(time,ddw_d_cmd(:,i));
hold on;
plot(time,ddw_d_ori(:,i));
plot(time,ddw_d_qpRes(:,i));
legend('ddw-d-cmd','ddw-d-ori','ddw-d-qp')
end


figure();
subplot(2,1,1)
plot(time,peW(:,1:3)-xCoM);
legend('peW-xCoM-x','peW-xCoM-y','peW-xCoM-z');
ylabel('right leg')
subplot(2,1,2)
plot(time,peW(:,4:6)-xCoM);
legend('peW-xCoM-x','peW-xCoM-y','peW-xCoM-z');
ylabel('left leg')


figure();
subplot(2,1,1)
plot(time,pe_Body_Accumu(:,1:3));
hold on;
plot(time,pe_Body_Accumu_OriRec(:,1:3));
legend('peBR-Body-Accumu-x','peBR-Body-Accumu-y','peBR-Body-Accumu-z', ...
    'peBR-Body-Accumu-ori-x','peBR-Body-Accumu-ori-y','peBR-Body-Accumu-ori-z');
subplot(2,1,2)
plot(time,pe_Body_Accumu(:,4:6));
hold on;
plot(time,pe_Body_Accumu_OriRec(:,4:6));
legend('peBL-Body-Accumu-x','peBL-Body-Accumu-y','peBL-Body-Accumu-z', ...
    'peBL-Body-Accumu-ori-x','peBL-Body-Accumu-ori-y','peBL-Body-Accumu-ori-z');


figure("Name",'ufe');
subplot(2,2,1)
plot(time,ufe_Ori(:,1:3));
hold on;
plot(time,ufe_Now(:,1:3));
legend('feR-x-ori','feR-y-ori','feR-z-ori','feR-x-Now','feR-y-Now','feR-z-Now');
subplot(2,2,2)
plot(time,ufe_Ori(:,4));
hold on;
plot(time,ufe_Now(:,4));
legend('feR-z-ori','feR-z-Now');
subplot(2,2,3)
plot(time,ufe_Ori(:,5:7));
hold on;
plot(time,ufe_Now(:,5:7));
legend('feL-x-ori','feL-y-ori','feL-z-ori','feL-x-Now','feL-y-Now','feL-z-Now');
subplot(2,2,4)
plot(time,ufe_Ori(:,8));
hold on;
plot(time,ufe_Now(:,8));
legend('feL-z-ori','feL-z-Now');










