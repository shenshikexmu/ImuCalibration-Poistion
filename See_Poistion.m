function See_Poistion( positionW,Q)
% show the gesture and Poistion of sensor
% data is raw data ;  
% author  Zhang Xin


 

x_max=max(positionW(:,1));
y_max=max(positionW(:,2));
z_max=max(positionW(:,3));
x_min=min(positionW(:,1));
y_min=min(positionW(:,2));
z_min=min(positionW(:,3));



figure('NumberTitle', 'off', 'Name', 'sensor attitude ');
set(gcf, 'doublebuffer', 'on');
% writerObj=VideoWriter('J:\out.avi');
% open(writerObj);
for i = 1:10:size(positionW,1)     %show sensor attitude
   
    
    R=quatern2rotMat(Q(i,:))*0.1;             % only gyro attitude in using RK4 updata  

    r1=R(1,:);                          
    g1=R(2,:);
    b1=R(3,:);
  
    
    plot3([positionW(i,1),r1(1)+positionW(i,1)],[positionW(i,2),r1(2)+positionW(i,2)],...
          [positionW(i,3),r1(3)+positionW(i,3)],'r-','LineWidth',2);
    hold on
    plot3([positionW(i,1),g1(1)+positionW(i,1)],[positionW(i,2),g1(2)+positionW(i,2)],...
          [positionW(i,3),g1(3)+positionW(i,3)],'g-','LineWidth',2);
    hold on
    plot3([positionW(i,1),b1(1)+positionW(i,1)],[positionW(i,2),b1(2)+positionW(i,2)],...
          [positionW(i,3),b1(3)+positionW(i,3)],'b-','LineWidth',2);
    hold on
    plot3(positionW(1:i,1),positionW(1:i,2),positionW(1:i,3),'k-');

    hold off
        
    axis equal
    set(gca,'XLim',[x_min-0.1 x_max+0.1]);
    set(gca,'YLim',[y_min-0.1 y_max+0.1]);
    set(gca,'ZLim',[z_min-0.1 z_max+0.1]);
    
    xlabel('X');  
    ylabel('Y');  
    zlabel('Z');  
    
    title(['i=' num2str(i)]);
    view(-48,18);
    
    drawnow
    
%     %%%%
%     frame=getframe;
%     writeVideo(writerObj,frame);
end
%close(writerObj);


% end
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












