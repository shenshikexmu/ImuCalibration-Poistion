function [positionW,Q_RK4]=Poistion(data,Ta,Ka,Ba,Tg,Kg,Bg,Tm2a,Bm)

% Poistion of sensor
% data is raw data ;  
% author  Zhang Xin

m=size(data,1);

%====================================================
t(1)=0;

for i=1:m
    
     data(i,2:4)=(Ta*Ka*(data(i,2:4)'+Ba))';
     data(i,5:7)=(Tg*Kg*(data(i,5:7)'+Bg))';
     data(i,8:10)=(Tm2a*(data(i,8:10)'+Bm))';
    
     norm_a(i,1)=norm(data(i,2:4));
     norm_g(i,1)=norm(data(i,5:7));
     t(1)=0;
     if i>1
         t(i)=data(i,1)-data(i-1,1);
     end
    if i==1 
        Q_RK4(i,:)=accMeg2qRichard(mean(data(1:1,:),1));  % only gyro 
    
    elseif i==2 
        Q_RK4(i,:)=attitude_update_RK4(Q_RK4(i-1,:)',t(i),data(i-1,5:7)',data(i,5:7)')';
    else

         Q_RK4(i,:)=attitude_update_RK4(Q_RK4(i-1,:)',t(i),data(i-1,5:7)',data(i,5:7)')';
    end
    
    accW(i,:)=accWorldframe(quatern2rotMat(Q_RK4(i,:)),data(i,:))';   %acceleration in world coordinata
    
end 

 [PP]=FindFixPointInSequence( Q_RK4(:,1),0.0002);

 speedW=zeros(m,3);

for ii=1:size(PP,1)
    
    for j=PP(ii,1):PP(ii,2)
        
        speedW(j,:)=speedW(j-1,:)+(accW(j-1,:)+accW(j,:))*t(j)/2;
    end 
    speedError=speedW(PP(ii,2),:)/(PP(ii,2)-PP(ii,1)+1);
   
    for j=PP(ii,1):PP(ii,2)
        
        speedW(j,:)=speedW(j-1,:)+(accW(j-1,:)+accW(j,:))*t(j)/2-speedError;
    end
    
end

figure 
ax(1)=subplot(7,1,1);
plot(accW(:,1));
ax(2)=subplot(7,1,2);
plot(accW(:,2));
ax(3)=subplot(7,1,3);
plot(accW(:,3));
ax(4)=subplot(7,1,4);
plot(speedW(:,1));
ax(5)=subplot(7,1,5);
plot(speedW(:,2));
ax(6)=subplot(7,1,6);
plot(speedW(:,3));
ax(7)=subplot(7,1,7);
plot(norm_a);
for j=1:size(PP,1)
   hold on
   plot(PP(j,1),norm_a(PP(j,1),1),'ro');
   hold on
   plot(PP(j,2),norm_a(PP(j,2),1),'ko');
end
linkaxes(ax,'x');


positionW=[0,0,0];
for i=2:m
    positionW(i,:)=positionW(i-1,:)+(speedW(i,:)+speedW(i-1,:))*t(i)/2;
end




end

function q = accMeg2qRichard(data)


vX=cross(data(1,8:10),data(1,2:4));
vX=vX/norm(vX);
vY=cross(data(1,2:4),vX);
vY=vY/norm(vY);

qX = qUtoV(vX,[1,0,0]);

y= qMultiVec(vY, qX);
qY = qUtoV(y,[0,1,0]);

qx=[-qX(1),qX(2:4)];
qy=[-qY(1),qY(2:4)];

q =qMultiQ(qx,qy);
q=[q(1),-q(2:4)];
if q(1)<0
    q=-q;
end
end


function [qq]=qMultiQ(p,q)   %p*q
qq=[...
        p(1) * q(1) - p(2) * q(2) - p(3) * q(3) - p(4) * q(4)...
       ,p(2) * q(1) + p(1) * q(2) - p(4) * q(3) + p(3) * q(4)...
       ,p(3) * q(1) + p(4) * q(2) + p(1) * q(3) - p(2) * q(4)...
       ,p(4) * q(1) - p(3) * q(2) + p(2) * q(3) + p(1) * q(4)  ];

end

function q = qUtoV(u, v)        %two vetor rotation to quaternions
nu = u/norm(u);
nv = v/norm(v);

if (u*v' == -1)
    q = [0, [1,0,0]];
end
    half = (nu + nv)/norm(nu + nv);
    q = [nu*half',cross(nu, half)];
end

function [vector]=qMultiVec(vec,q)  %sensor frame to world frame
x = q(2);
y = q(3);
z = q(4);
w = q(1);

vecx = vec(1);
vecy = vec(2);
vecz = vec(3);

x_ =  w * vecx  +  y * vecz  -  z * vecy;
y_ =  w * vecy  +  z * vecx  -  x * vecz;
z_ =  w * vecz  +  x * vecy  -  y * vecx;
w_ = -x * vecx  -  y * vecy  -  z * vecz;

vector = [x_ * w  +  w_ * -x  +  y_ * -z  -  z_ * -y ...
    , y_ * w  +  w_ * -y  +  z_ * -x  -  x_ * -z ...
    , z_ * w  +  w_ * -z  +  x_ * -y  -  y_ * -x ...
    ];

end

function R = quatern2rotMat(q)
    [rows cols] = size(q);
    R = zeros(3,3, rows);
    R(1,1,:) = 2.*q(:,1).^2-1+2.*q(:,2).^2;
    R(1,2,:) = 2.*(q(:,2).*q(:,3)+q(:,1).*q(:,4));
    R(1,3,:) = 2.*(q(:,2).*q(:,4)-q(:,1).*q(:,3));
    R(2,1,:) = 2.*(q(:,2).*q(:,3)-q(:,1).*q(:,4));
    R(2,2,:) = 2.*q(:,1).^2-1+2.*q(:,3).^2;
    R(2,3,:) = 2.*(q(:,3).*q(:,4)+q(:,1).*q(:,2));
    R(3,1,:) = 2.*(q(:,2).*q(:,4)+q(:,1).*q(:,3));
    R(3,2,:) = 2.*(q(:,3).*q(:,4)-q(:,1).*q(:,2));
    R(3,3,:) = 2.*q(:,1).^2-1+2.*q(:,4).^2;
end

function [accW]=accWorldframe(R,data)

accW=(R'*data(1,2:4)'-[0;0;9.8]);


end


function [Qk_plus1]=attitude_update_RK4(Qk,dt,gyro0,gyro1)
% RK4
% conference: A Robust and Easy to implement method for imu
% calibration without External Equipments

q_1=Qk;
k1=(1/2)*omegaMatrix(gyro0)*q_1;
q_2=Qk+dt*(1/2)*k1;
k2=(1/2)*omegaMatrix((1/2)*(gyro0+gyro1))*q_2;
q_3=Qk+dt*(1/2)*k2;
k3=(1/2)*omegaMatrix((1/2)*(gyro0+gyro1))*q_3;
q_4=Qk+dt*k3;
k4=(1/2)*omegaMatrix(gyro1)*q_4;
Qk_plus1=Qk+dt*(k1/6+k2/3+k3/3+k4/6);
Qk_plus1=Qk_plus1/norm(Qk_plus1);

if Qk_plus1(1)<0
    Qk_plus1=-Qk_plus1;
end

end

function [omega]=omegaMatrix(data)

% wx=data(1)*pi/180;
% wy=data(2)*pi/180;
% wz=data(3)*pi/180;
wx=data(1);
wy=data(2);
wz=data(3);

omega=[0  , -wx , -wy , -wz ;...
       wx ,  0  ,  wz , -wy ;...
       wy , -wz ,  0  ,  wx ;...
       wz ,  wy , -wx ,  0   ];

end

function  [PP]=FindFixPointInSequence(sequence,threshold)

sequence=abs([0;diff(sequence(:,1))]);
% figure
% plot(sequence)

stationary = sequence < threshold;
P(:,1) = find([0; diff(stationary)] == -1);
P(:,2) = find([0; diff(stationary)] == 1);

j=1;
PP(1,1)=P(1,1);

for i=2:size(P,1)
    
    if P(i,1)-P(i-1,2)>50
        PP(j,2)=P(i-1,2);
       
        j=j+1;
        PP(j,1)=P(i,1);
      
    end
    if i==size(P,1)
        PP(j,2)=P(end,2);
       
    end
end

% for j=1:size(PP,1)
%    hold on
%    plot(PP(j,1),sequence(PP(j,1)),'ro');
%    hold on
%    plot(PP(j,2),sequence(PP(j,2)),'ko');
% end

end
