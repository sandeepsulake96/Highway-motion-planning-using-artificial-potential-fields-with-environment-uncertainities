% road mesh
 

syms X Y;

%
k_tar = 10;
k_b1 = 0.7;
k_b2 = 0.7;
k_c = 35 ;
si_c =1;
k_obs= 100;
sx=100;   %70
sy= 1;   %1.4
k1= 0.005;
k2=0.005;
gamma =0;
k_edge1 =4.7;
k_edge2 =0.08;

% Target position (for target attraction)
x_tar = 950;
y_tar = 5.5;
Y1=8;   %left boundary
YC=4;   %center line
Y2=0;   %right boundary

% eq 1  (edge potential)
d1 = (Y-Y1/2); d2= Y-Y2/2;
u_edge1 = -k_edge1*(-exp(-d1)+1);
u_edge2 = -k_edge2*(-exp(d2)+1);
f1 = u_edge1+u_edge2;

% eq 2 (centerline potential)
dc = Y-YC;
f2 =  k_c*exp(-(dc.^2)./ (2*si_c^2));

%eq 3 (target potential)
%f3 = 1/200*( (X - x_tar).^2 + (1/50)*(Y- y_tar).^2 )
%f3 = -1000*exp(-((((X-x_tar).^2./100000)) + (((Y-y_tar).^2./100))))
f3= -1.5*(X-x_tar);

%eq 4 (obstacle potential)
x_obs1 = 700;
y_obs1 = 5.5;

x_obs2 = 300;
y_obs2 = 2.5;
v=0;
v_obs =5;
f4 = k_obs*exp(- ((((X-x_obs1).^2)./sx^2) + (((Y-y_obs1).^2)./sy^2))) %+ gamma*((X-x_obs1).^2./sx^2)*(k1*v+k2*(v-v_obs)));

f5 = k_obs*exp(- ((((X-x_obs2).^2)./sx^2) + (((Y-y_obs2).^2)./sy^2)))
% total potential
f=   f1+f2+f3+f4+f5;


% Path Planning using gradient descent

start = [10,2.5];
goal = [x_tar,y_tar];
iter = 400;
route = grad_desc(start,goal,f,iter,3);


% Plotting
x1 = 1:10:1000;
y1 = 1:0.1:7;
[xx,yy]= meshgrid(x1,y1);

fval = double((subs(f,{X,Y},{xx,yy})));
route_height = double((subs(f,{X,Y},{route(:,1),route(:,2)})));

%surface plot with route
figure(1)
surf(xx,yy,fval);
hold on
plot3(route(:,1),route(:,2),route_height,'g','LineWidth',3)
plot3(route(1,1),route(1,2),route_height(1),'r*','LineWidth',5)
plot3(route(end,1),route(end,2),route_height(end),'r*','LineWidth',5)




% 2d plot with route

figure(2)
%contour(x1,y1,fval, 'Fill', 'On');
%hold on;
yline(6.7,'-','LineWidth',4)
hold on
yline(1.2,'-','LineWidth',4)
hold on
yline(4,'--','LineWidth',3)
hold on
plot(route(:,1),route(:,2),'linewidth',4,'Color','r');

ylim([1,7]);
set(gcf,'position',[x1(1),y1(1),1000,150])

%% Calculate reference poses for vehicle tracking and control
 
%thetaRef = zeros(length(route(:,1)),1);

%gradbp = linspace(0,route(end))
xRef2= route(:,1);
yRef2= route(:,2);

speed =10; %longitudinal speed

L = 3; % bicycle length
ld = 5; % lookahead distance
Ts = 16; % simulation time

sim('pure_pursuit_tracking.slx');
% X_o = refPose(1,1); % initial vehicle position
% Y_o = -refPose(1,2); % initial vehicle position 
% psi_o = 0; % initial yaw angle


figure(3)
plot(xRef2,yRef2,'LineWidth',2)
xlim([0 1000]);
ylim([2 6]);
hold on
plot(x_track,y_track,'--g',LineWidth=2);

%error plot

y_track_interp = interp1(x_track,y_track,xRef2);

figure(4)
plot(xRef2,yRef2-y_track_interp,'LineWidth',2);
xlabel('X')
ylabel('Lateral Error')


figure(5)
plot(yaw,'LineWidth',2);
xlabel('X')
ylabel('Yaw[rad]')


 %%
% for i = 2:length(route(:,1))
%     thetaRef(i,1) = atan2d((yRef2(i)-yRef2(i-1)),(xRef2(i)-xRef2(i-1)));
% end
% 
% thetaRef(1) = thetaRef(2);
% thetaRefs = smooth(route(:,1),thetaRef);
% 
% psi_o = thetaRefs(2)*(pi/180);
% 
% refpos = [route,thetaRefs];

%%

%%%
% route=start;
% point_on_route=start;
% 
% % Gradient Computation:
% df_dx = diff(f, X);
% df_dy = diff(f, Y);
% J = [subs(df_dx,[X,Y], [point_on_route(1),point_on_route(2)]) subs(df_dy, [X,Y], [point_on_route(1),point_on_route(2)])]; % Gradient
% S = -(J); % Search Direction
% hx=1;
% hy=0.01;
% e=0.1;
% while norm(J) > e 
% %while(max_iter >0)
% 
%     %I = [x(i),y(i)];
%     
%     %if (norm(goal-point_on_route)< dist_tol)
%        % break;
%     %end
%     
% %     syms h; % Step size
% %     g = subs(f, [X,Y], [x(i)+S(1)*h,y(i)+h*S(2)]);
% %     dg_dh = diff(g,h);
% %     h = solve(dg_dh, h); % Optimal Step Length
%     
%     new_route_x = point_on_route(1)+hx*S(1);
%     new_route_y = point_on_route(2)+hy*S(2);
%     
%     point_on_route = [new_route_x,new_route_y];
%     route = [route;point_on_route];
%     %max_iter = max_iter-1;
% 
%     %x(i+1) = I(1)+h*S(1); % Updated x value
%     %y(i+1) = I(2)+0.01*S(2); % Updated y value
%     %i = i+1;
%     J = [subs(df_dx,[X,Y], [point_on_route(1),point_on_route(2)]) subs(df_dy, [X,Y], [point_on_route(1),point_on_route(2)])]; % Updated Gradient
%     S = -(J); % New Search Direction
% end
% 
% %%
% x(1) = start(1);
% y(1) = start(2);
% 
% e = 0.1; % Convergence Criteria
% i = 1; % Iteration Counter
% % Gradient Computation:
% df_dx = diff(f, X);
% df_dy = diff(f, Y);
% J = [subs(df_dx,[X,Y], [x(1),y(1)]) subs(df_dy, [X,Y], [x(1),y(1)])]; % Gradient
% S = -(J); % Search Direction
% 
% while norm(J) > e 
%     I = [x(i),y(i)]';
% %     syms h; % Step size
% %     g = subs(f, [X,Y], [x(i)+S(1)*h,y(i)+h*S(2)]);
% %     dg_dh = diff(g,h);
% %     h = solve(dg_dh, h); % Optimal Step Length
%     h=1;
%     x(i+1) = I(1)+h*S(1); % Updated x value
%     y(i+1) = I(2)+0.01*S(2); % Updated y value
%     i = i+1;
%     J = [subs(df_dx,[X,Y], [x(i),y(i)]) subs(df_dy, [X,Y], [x(i),y(i)])]; % Updated Gradient
%     S = -(J); % New Search Direction
%     
% end
% %Result Table:
% % Iter = 1:i;
% % X_coordinate = x';
% % Y_coordinate = y';
% % Iterations = Iter';
% % T = table(Iterations,X_coordinate,Y_coordinate);
% % Plots:
% fcontour(f, 'Fill', 'On');
% hold on;
% plot(x,y,'*-r');
% 
% % Output:
% %fprintf('Initial Objective Function Value: %d\n\n',subs(f,[X,Y], [x(1),y(1)]));
% % if (norm(J) < e)
% %     fprintf('Minimum succesfully obtained...\n\n');
% % end
% % fprintf('Number of Iterations for Convergence: %d\n\n', i);
% % fprintf('Point of Minima: [%d,%d]\n\n', x(i), y(i));
% % fprintf('Objective Function Minimum Value Post-Optimization: %d\n\n', subs(f,[X,Y], [x(i),y(i)]));
% % disp(T);


%% obstacle potential and animation
% v_obs = 10;
% dt= 0.1;
% x_obs1 = 70;
% y_obs1 = 2;
% 
% x_obs2 = 10;
% y_obs2 = -2;
% 
% v=0;
% while x_obs1< x_tar
% 
%     x_obs1= x_obs1+v_obs*dt;
%     %y_obs1 = 2;
% 
% 
%     x_obs2= x_obs2+v_obs*dt;
%     %y_obs2 = -2;
%     
%     u_car1 = k_obs*exp(- ((((X-x_obs1).^2)./sx^2) + (((Y-y_obs1).^2)./sy^2)) + gamma*((x-x_obs1).^2./sx^2)*(k1*v+k2*(v-v_obs)));
%     u_car2 = k_obs*exp(- ((((X-x_obs2).^2)./sx^2) + (((Y-y_obs2).^2)./sy^2)) + gamma*((x-x_obs2).^2./sx^2)*(k1*v+k2*(v-v_obs)));
% % 
%     figure(1)
%     surf(X,Y,u_car1+u_car2+u_r+u_tar)
%     view([60 -150])
% end
% 

%% Plan route
% start = [10,3];
% goal = [x_tar,y_tar];
% 
% route = GradientBasedPlanner (f, start, goal, 100,x,y);
% 


%% quiver plot
% 
% [gx, gy] = gradient (-fval);
% skip = 2;
% 
% figure;
% 
% xidx = 1:skip:500;
% yidx = 1:skip:13;
% 
% quiver (X(yidx,xidx), Y(yidx,xidx), gx(yidx,xidx), gy(yidx,xidx), 0.4);
% 
% axis ([1 1000 1 8]);
% 
% hold on;
% 
% ps = plot(start(1), start(2), 'r.', 'MarkerSize', 30);
% pg = plot(goal(1), goal(2), 'g.', 'MarkerSize', 30);
% p3 = plot (route(:,1), route(:,2), 'r', 'LineWidth', 2);


%%











