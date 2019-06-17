load('Data.mat');
% data=cal_data;

[~,fix_point,rotation]=FindFixData2(data,30);

[Ta,Ka,Ba]=ICRA2014_acc(fix_point);

Bg=-mean(fix_point(:,4:6),1)';

n=size(rotation,1);

rotation{n+1}=Ta;
rotation{n+2}=Ka;
rotation{n+3}=Ba;
rotation{n+4}=Bg;

[Tg,Kg]=ICRA_2014_gyro(rotation);

[Tm2a,Bm,Vm,mag_strength]=Cal_mag4acc_frame(rotation,fix_point,Tg,Kg);
%%

[positionW,Q_RK4]=Poistion(data,Ta,Ka,Ba,Tg,Kg,Bg,Tm2a,Bm);

See_Poistion( positionW,Q_RK4);
