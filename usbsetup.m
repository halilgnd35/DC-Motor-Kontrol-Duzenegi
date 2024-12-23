%BASLANGIC
Ts_rec = 0.5e-3;
Ts_algo = 1e-3;
Ts =10e-3;
Ts_send = 0.5e-3;
T=Ts;
Td=0.01;
Ti=1;
K=1;


%KUBRA/ z-domain
% Ts_rec = 0.5e-3;
% Ts_algo = 1e-3;
% Ts =100e-3;
% Ts_send = 0.5e-3;
% T=Ts;
% Td=0.01;
% Ti=1;
% K=1;
% a0=1;
% PM=55;
% Gw1_amplitude=0.62;
% Gw1_amplitudeDb=20*log10(Gw1_amplitude);
% Gw1_phase=-172.7874;
% ww1=7.9176;
% teta=-180+PM-Gw1_phase;
% a1=(1-a0*Gw1_amplitude*0.6719)/(ww1*Gw1_amplitude*0.7406);
% b1=(0.6719-a0*Gw1_amplitude)/(ww1*0.7406);
% ww0=a0/a1;
% wwp=1/b1;
% num=[a1 a0];
% den=[b1 1];
% Dw=tf(num,den);
% Kd=a0*(wwp*(ww0+2/T))/(ww0*(wwp+2/T));
% z0=((2/T)-ww0)/((2/T)+ww0);
% zp=((2/T)-wwp)/((2/T)+wwp);
% num1=[Kd -Kd*z0];
% dem1=[1 -zp];
% Dz=tf(num1,dem1);