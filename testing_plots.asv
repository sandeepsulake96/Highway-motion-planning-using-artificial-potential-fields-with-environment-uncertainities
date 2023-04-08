syms X Y;

%
k_tar = 10;
k_b1 = 0.7;
k_b2 = 0.7;
k_c = 35 ;
si_c =1;
k_obs= 100;
sx= 60;   %70
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
x_obs1 = 500;
y_obs1 = 5.5;

x_obs2 = 300;
y_obs2 = 2.5;
v=0;
v_obs =5;

% factors to consider for obstacle potential. 

c1 = 0.8;
c2 = 0; %0.000025;
c3= 1.1;
mu=0.2;
M_v = 10000;

f4 = k_obs*exp(- ((((X-x_obs1).^2)./sx^2) + (((Y-y_obs2).^2)./sy^2)) + (c1*(1-mu)+c2*(M_v))*(((X-x_obs1).^2)./sx^2) + c3*(1-0.15)*((Y-y_obs2).^2)./sy^2)    %+ gamma*((X-x_obs1).^2./sx^2)*(k1*v+k2*(v-v_obs)));

f5 = k_obs*exp(- ((((X-x_obs2).^2)./sx^2) + (((Y-y_obs2).^2)./sy^2)))
% total potential
f=   f1+f2+f3+f4+f5;





%%plotting for obstacle potential

x1 = 1:5:1000;
y1 = 1:0.1:7;
[xx,yy]= meshgrid(x1,y1);
f11=f4+f1+f2
fval1 = double((subs(f11,{X,Y},{xx,yy})));
figure(7)
surf(xx,yy,fval1);
shading interp


%% 2D PLOT guassian distribution


ff1 = k_obs*exp(- ((((x1-x_obs1).^2)./30^2))  + (c1*(1-mu)+c2*(1500))*(((x1-x_obs1).^2)./sx^2) )    %+ gamma*((X-x_obs1).^2./sx^2)*(k1*v+k2*(v-v_obs)));

ff2 = k_obs*exp(- ((((x1-x_obs1).^2)./60^2))  + (c1*(1-mu)+c2*(5000))*(((x1-x_obs1).^2)./sx^2))    %+ gamma*((X-x_obs1).^2./sx^2)*(k1*v+k2*(v-v_obs)));
ff3 = k_obs*exp(- ((((x1-x_obs1).^2)./100^2))  + (c1*(1-mu)+c2*(10000))*(((x1-x_obs1).^2)./sx^2))    %+ gamma*((X-x_obs1).^2./sx^2)*(k1*v+k2*(v-v_obs)));

y11=0:0.1:8
fy1= k_obs*exp(- ((y11-y_obs2).^2)./sy^2 + c3*(1-0.2)*((y11-y_obs2).^2)./sy^2)
fy2= k_obs*exp(- ((y11-y_obs2).^2)./sy^2 + c3*(1-0.5)*((y11-y_obs2).^2)./sy^2)
fy3= k_obs*exp(- ((y11-y_obs2).^2)./sy^2 + c3*(1-0.8)*((y11-y_obs2).^2)./sy^2)


figure(8)
plot(y11,fy1,'LineWidth',2)
hold on
plot(y11,fy2,'LineWidth',2)
hold on
plot(y11,fy3,'LineWidth',2)
grid on
xlabel('Y[m]')
ylabel('Obstacle vehicle potential (Lateral) ')

legend('\mu=0.2','\mu=0.5','\mu=0.8')
%legend('\mu=0.2, M_v=1500 kg','\mu=0.2, M_v=5000 kg','\mu=0.2, M_v=10000 kg')
legend('\sigma_x=30','\sigma_x=60','\sigma_x=90')



%% Path Planning using gradient descent

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





