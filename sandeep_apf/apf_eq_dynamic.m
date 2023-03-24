% road mesh
 
% [X,Y] = meshgrid(x,y);
% %F = zeros*X;
% %surf(X,Y,F)
clear all
syms X Y;

%
k_tar = 10;
k_b1 = 0.7;
k_b2 = 0.7;
k_c = 35 ;
si_c =1;
k_obs= 100;
sx=40;   %70
sy= 1;   %1.4
k1= 0.005;
k2=0.005;
gamma =0;
k_edge1 =4.7;
k_edge2 =0.08;

% Target position (for target attraction)
x_tar = 900;
y_tar = 5.5;
Y1=8;  %left boundary
YC=4;   %center line
Y2=0;   % right boundary

x_obs1 = 200;
y_obs1 = 5.5;

x_obs2 = 200;
y_obs2 = 2.5;

x_obs3 = 600;
y_obs3 = 5.5;

v=0;
v_obs =2;
dt=1;


% eq 1  (edge potential)
d1 = (Y-Y1/2); d2= Y-Y2/2;
u_edge1 = -k_edge1*(-exp(-d1)+1);
u_edge2 = -k_edge2*(-exp(d2)+1);
f1 = u_edge1+u_edge2;

% eq 2 (centerline potential)
dc = Y-YC;
f2 =  k_c*exp(-(dc.^2)./ (2*si_c^2));

% eq 3 (target potential)
%f3 = 1/200*( (X - x_tar).^2 + (1/50)*(Y- y_tar).^2 )
%f3 = -1000*exp(-((((X-x_tar).^2./100000)) + (((Y-y_tar).^2./100))))
f3= -4*(X-x_tar);


start = [10,2.5];
final_route= start;
goal = [x_tar,y_tar];
x1 = 1:2:1000;
y1 = 1:0.5:7;
[xx,yy]= meshgrid(x1,y1);

i=1;
while final_route(end,1)< x_tar


    x_obs1(i+1)= x_obs1(i)+v_obs*dt; %only  longitudinal distance change
    x_obs2(i+1)= x_obs2(i)+v_obs*dt; %only  longitudinal distance change
    x_obs3(i+1)= x_obs3(i)+v_obs*dt; %only  longitudinal distance change
    %y_obs1(i+1)= y_obs1(i)-0.04*dt;

    %eq 4 (obstacle potential)
    f4 = k_obs*exp(- ((((X-x_obs1(i)).^2)./sx^2) + (((Y-y_obs1).^2)./sy^2))); %+ gamma*((X-x_obs1).^2./sx^2)*(k1*v+k2*(v-v_obs)));
    f5 = k_obs*exp(- ((((X-x_obs2(i)).^2)./sx^2) + (((Y-y_obs2).^2)./sy^2)));
    f6 = k_obs*exp(- ((((X-x_obs3(i)).^2)./sx^2) + (((Y-y_obs3).^2)./sy^2)));



    % total potential
    f=f1+f2+f3+f4+f5+f6;

    next = grad_desc(final_route(i,:),goal,f,1,3); %grad_desc(start,goal,f,max_iter,dist_tol)
    final_route(i+1,:) = next(end,:);
    
    %Plotting
    fval = double((subs(f,{X,Y},{xx,yy})));
    route_height = double((subs(f,{X,Y},{final_route(i+1,1),final_route(i+1,2)})));

    figure(1)
    contour(x1,y1,fval, 'Fill', 'On');
    hold on;
    plot(final_route(i+1,1),final_route(i+1,2),'b.','LineWidth', 2, 'MarkerSize', 40);
    hold on
    plot(x_obs1,y_obs1,'r.','LineWidth', 2, 'MarkerSize', 40)
    hold on
    plot(x_obs2,y_obs2,'r.','LineWidth', 2, 'MarkerSize', 40)
    hold on
    plot(x_obs3,y_obs3,'r.','LineWidth', 2, 'MarkerSize', 40)
    hold on
    set(gcf,'position',[x1(1),y1(1),1000,150])

    %plot3(route(1,1),route(1,2),route_height(1),'r*','LineWidth',5)
    %plot3(route(end,1),route(end,2),route_height(end),'r*','LineWidth',5)
    i=i+1;
end


%%
% 2d plot with route
route_height = double((subs(f,{X,Y},{final_route(:,1),final_route(:,2)})));
figure(2)
contour(x1,y1,fval, 'Fill', 'On');
hold on;
plot(final_route(:,1),final_route(:,2),'linewidth',4,'Color','r');

figure(3)
surf(xx,yy,fval);
hold on
plot3(final_route(:,1),final_route(:,2),route_height,'g','LineWidth',3)


%% Animation 
%Plotting obstacle and route points
figure(4)
%contour(x1,y1,fval, 'Fill', 'On');
yline(6.7,'-','LineWidth',4)
hold on
yline(1.2,'-','LineWidth',4)
hold on
yline(4,'--','LineWidth',3)
hold on
set(gcf,'position',[x1(1),y1(1),1000,150]);
xlim([1 1000])
ylim([1,7]);

hold on
% 
for i=1:length(x_obs1)

    plot(final_route(i,1),final_route(i,2),'b.','LineWidth', 2, 'MarkerSize', 20);
    hold on
    h1=plot(x_obs1(i),y_obs1(i),'r.','LineWidth', 2, 'MarkerSize', 20);
    hold on
    h2= plot(x_obs2(i),y_obs2,'r.','LineWidth', 2, 'MarkerSize', 20);
    hold on
    h3= plot(x_obs3(i),y_obs3,'r.','LineWidth', 2, 'MarkerSize', 20);
    pause(0.1)
    F(i) = getframe(gcf);
    delete(h1);
    delete(h2);
    delete(h3);

end



%%  Play movie from the recorded F data
x1 = 1:2:1000;
y1 = 1:0.5:7;
[xx,yy]= meshgrid(x1,y1);
fig = figure;
set(gcf,'position',[x1(1),y1(1),1000,150])
xlim([1 1000])
ylim([1,7]);
movie(fig,F,1)


%% writing video file


writerObj = VideoWriter('3_obs.gif');
writerObj.FrameRate = 10;
open(writerObj);
% write the frames to the video
for i=1:length(F)
    frame = F(i) ;    
    writeVideo(writerObj, frame);
end
% close the writer object
close(writerObj)


%%











