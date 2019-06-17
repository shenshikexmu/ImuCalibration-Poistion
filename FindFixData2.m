function [PP,fix_point,rotation]=FindFixData2(cal,threshold)

% author  Zhang Xin

n=size(cal,1);


for i=1:n
    norm_gyro(i,1)=norm(cal(i,5:7));

end

stationary = norm_gyro < threshold;
P(:,1) = find([0; diff(stationary)] == -1);
P(:,2) = find([0; diff(stationary)] == 1);

margin=5;
margin2=35;
j=1;
PP(1,1)=P(1,1)-margin;
fix_point(j,:)=mean(cal(1:PP(j,1)-margin2,2:10),1);
for i=2:size(P,1)
    
    if P(i,1)-P(i-1,2)>50
        PP(j,2)=P(i-1,2)+margin;
        rotation{j,1}=cal(PP(j,1)-margin2:PP(j,2)+margin2,:);
        j=j+1;
        PP(j,1)=P(i,1)-margin;
        fix_point(j,:)=mean(cal(PP(j-1,2)+margin2:PP(j,1)-margin2,2:10),1);
    end
    if i==size(P,1)
        PP(j,2)=P(end,2)+margin;
        rotation{j,1}=cal(PP(j,1)-margin2:PP(j,2)+margin2,:);
        fix_point(j+1,:)=mean(cal(PP(j,2)+margin2:end,2:10),1);
    end
end


figure 

plot(1:n,norm_gyro,'b')
for j=1:size(PP,1)
   hold on
   plot(PP(j,1),norm_gyro(PP(j,1)),'ro');
   hold on
   plot(PP(j,2),norm_gyro(PP(j,2)),'ko');
end


end
