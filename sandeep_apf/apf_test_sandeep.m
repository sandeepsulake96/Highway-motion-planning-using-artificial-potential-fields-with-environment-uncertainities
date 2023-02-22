% road mesh
x = 1:1:200;
y = -4:0.1:4;
[X,Y] = meshgrid(x,y);
%F = zeros*X;
%surf(X,Y,F)


%% Parameters

k_tar = 0.25
k_b1 = 0.5;
k_b2 = 0.5;
k_c = 6.5;
si_c =0.5;
k_obs= 15;
sx=14;   %10
sy= 0.7;   %1.4
k1= 0.005;
k2=0.005;
gamma =0;


% Target position (for target attraction)
x_tar = 190;
y_tar = 2;
Y1=4.1;  %left boundary
YC=0;   %center line
Y2=-4.1;   % right boundary


%% Target potential

u_tar = -k_tar*(X-x_tar);

figure(1)
surf(X,Y,u_tar)



%% Road Potential

% edge potential
d1 = (Y-Y1); d2= Y-Y2;
u_e = (k_b1./d1.^2) + (k_b2./d2.^2);      %k_edge*(-exp(Y-Y1)+1);
%u_e=u_e';                                 % u_edge2 = k_edge*(-exp(Y-Y2)+1);
%u_e(1,:)=51;
%u_e(end,:)= 51;


% centerline potential
dc = Y-YC;
u_c =  k_c*exp(-(dc.^2)./ (2*si_c^2));
%u_c = u_c';r

%total road potential
u_r =u_e+u_c;
surf(X,Y,u_r);

%%    Obstacle 

%testing static obstacle potential
x_obs = 70;
y_obs = 2;
v=0;
v_obs =5;
u_car1 = k_obs*exp(- ((((X-x_obs).^2)./sx^2) + (((Y-y_obs).^2)./sy^2)) + gamma*((X-x_obs).^2./sx^2)*(k1*v+k2*(v-v_obs)));



%% Total Potential

f = u_r+u_tar+u_car1;


%plotting total potential (obstacle + Road+ target)
surf(X,Y,f)

%% obstacle potential and animation
% v_obs = 10;
% dt= 0.1;
% x_obs = 70;
% y_obs = 2;
% v=0;
% while x_obs< 190
% 
%     x_obs= x_obs+v_obs*dt;
%     y_obs = 2;
%     u_car1 = k_obs*exp(- ((((X-x_obs).^2)./sx^2) + (((Y-y_obs).^2)./sy^2)) + gamma*((x-x_obs).^2./sx^2)*(k1*v+k2*(v-v_obs)));
%     figure(1)
%     surf(X,Y,u_car1+u_r+u_tar)
% end


%% Plan route
start = [71,2];
goal = [x_tar,y_tar];

route = GradientBasedPlanner (f, start, goal, 100);



%% quiver plot

[gx, gy] = gradient (-f);
skip = 2;

figure;

xidx = 1:skip:200;
yidx = 1:skip:81;

quiver (X(yidx,xidx), Y(yidx,xidx), gx(yidx,xidx), gy(yidx,xidx), 0.4);

axis ([1 200 1 30]);

hold on;

ps = plot(start(1), start(2), 'r.', 'MarkerSize', 30);
pg = plot(goal(1), goal(2), 'g.', 'MarkerSize', 30);
p3 = plot (route(:,1), route(:,2), 'r', 'LineWidth', 2);


%%











