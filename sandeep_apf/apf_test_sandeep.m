% road mesh
x = 1:2:400;
y = 0:0.5:8;
[X,Y] = meshgrid(x,y);
%F = zeros*X;
%surf(X,Y,F)


%% Parameters

k_tar = 10;
k_b1 = 0.7;
k_b2 = 0.7;
k_c = 100;
si_c =1;
k_obs= 15;
sx=14;   %10
sy= 0.7;   %1.4
k1= 0.005;
k2=0.005;
gamma =0;
k_edge1 =3;
k_edge2 =0.05;

% Target position (for target attraction)
x_tar = 390;
y_tar = 4;
Y1=8;  %left boundary
YC=4;   %center line
Y2=0;   % right boundary


% Target potential

%u_tar = -k_tar*(X-x_tar);
u_tar = 1/100*( (X - x_tar).^2 + (Y- y_tar).^2 )

%figure(1)
%surf(X,Y,u_tar)



% Road Potential

% edge potential
d1 = (Y-Y1/2); d2= Y-Y2/2;
%u_e = (k_b1./d1.^2) + (k_b2./d2.^2);   
u_edge1 = -k_edge1*(-exp(-d1)+1);
u_edge2 = -k_edge2*(-exp(d2)+1);
%u_e=u_e';                          u_edge2 = k_edge*(-exp(Y-Y2)+1);
%u_e(1,:)=51;
%u_e(end,:)= 51;

u_edge_total = u_edge1+u_edge2;

% centerline potential
dc = Y-YC;
u_c =  k_c*exp(-(dc.^2)./ (2*si_c^2));
%u_c = u_c';r

%total road potential
u_r =u_edge_total+u_c;
%surf(X,Y,u_edge_total);

surf(X,Y,u_c+u_edge_total+u_tar)

%%    Obstacle 

%testing static obstacle potential
x_obs1 = 70;
y_obs1 = 2;
v=0;
v_obs =5;
u_car1 = k_obs*exp(- ((((X-x_obs1).^2)./sx^2) + (((Y-y_obs1).^2)./sy^2)) + gamma*((X-x_obs1).^2./sx^2)*(k1*v+k2*(v-v_obs)));



%% Total Potential

%f = u_r+u_tar+u_car1*0;
f=u_c+u_edge_total+u_tar;

%plotting total potential (obstacle + Road+ target)
surf(X,Y,f)

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
start = [10,3];
goal = [x_tar,y_tar];

route = GradientBasedPlanner (f, start, goal, 100,x,y);



%% quiver plot

[gx, gy] = gradient (-f);
skip = 2;

figure;

xidx = 1:skip:200;
yidx = 1:skip:17;

quiver (X(yidx,xidx), Y(yidx,xidx), gx(yidx,xidx), gy(yidx,xidx), 0.4);

axis ([1 200 1 30]);

hold on;

ps = plot(start(1), start(2), 'r.', 'MarkerSize', 30);
pg = plot(goal(1), goal(2), 'g.', 'MarkerSize', 30);
p3 = plot (route(:,1), route(:,2), 'r', 'LineWidth', 2);


%%











