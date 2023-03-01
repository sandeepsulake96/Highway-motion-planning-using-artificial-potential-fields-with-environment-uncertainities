% road mesh
 
% [X,Y] = meshgrid(x,y);
% %F = zeros*X;
% %surf(X,Y,F)
syms X Y;

%%
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

%% eq 1  (edge potential)
d1 = (Y-Y1/2); d2= Y-Y2/2;
u_edge1 = -k_edge1*(-exp(-d1)+1);
u_edge2 = -k_edge2*(-exp(d2)+1);
f1 = u_edge1+u_edge2;

% eq 2 (centerline potential)
dc = Y-YC;
f2 =  k_c*exp(-(dc.^2)./ (2*si_c^2));
%eq 3 (target potential)
f3 = 1/100*( (X - x_tar).^2 + (Y- y_tar).^2 )

%
f=f1+f2+f3;

%f = X - Y + 2*X^2 + 2*X*Y + Y^2;
% Initial Guess:
%%
start = [10,3];
goal = [x_tar,y_tar];

x(1) = start(1);
y(1) = start(2);

e = 0.1; % Convergence Criteria
i = 1; % Iteration Counter
% Gradient Computation:
df_dx = diff(f, X);
df_dy = diff(f, Y);
J = [subs(df_dx,[X,Y], [x(1),y(1)]) subs(df_dy, [X,Y], [x(1),y(1)])]; % Gradient
S = -(J); % Search Direction

while norm(J) > e 
    I = [x(i),y(i)]';
%     syms h; % Step size
%     g = subs(f, [X,Y], [x(i)+S(1)*h,y(i)+h*S(2)]);
%     dg_dh = diff(g,h);
%     h = solve(dg_dh, h); % Optimal Step Length
    h=1;
    x(i+1) = I(1)+h*S(1); % Updated x value
    y(i+1) = I(2)+0.01*S(2); % Updated y value
    i = i+1;
    J = [subs(df_dx,[X,Y], [x(i),y(i)]) subs(df_dy, [X,Y], [x(i),y(i)])]; % Updated Gradient
    S = -(J); % New Search Direction
end
%Result Table:
% Iter = 1:i;
% X_coordinate = x';
% Y_coordinate = y';
% Iterations = Iter';
% T = table(Iterations,X_coordinate,Y_coordinate);
% Plots:
fcontour(f, 'Fill', 'On');
hold on;
plot(x,y,'*-r');

% Output:
%fprintf('Initial Objective Function Value: %d\n\n',subs(f,[X,Y], [x(1),y(1)]));
% if (norm(J) < e)
%     fprintf('Minimum succesfully obtained...\n\n');
% end
% fprintf('Number of Iterations for Convergence: %d\n\n', i);
% fprintf('Point of Minima: [%d,%d]\n\n', x(i), y(i));
% fprintf('Objective Function Minimum Value Post-Optimization: %d\n\n', subs(f,[X,Y], [x(i),y(i)]));
% disp(T);



%% 
x1 = 1:2:400;
y1 = 0:0.5:8;
[xx,yy]= meshgrid(x1,y1);
%surf(xx,yy,fval);

fval= zeros(length(y1),length(x1));
for i = 1:length(y1)
    for j = 1:length(x1)
        fval(i,j) = vpa(subs(f,[X,Y], [x1(j),y1(i)]));
       
    end
end
%%

% contour(x1,y1,fval,'Fill','On')
% hold on
% 


 surf(xx,yy,fval);hold on;
 plot(x,y,'r-');


 
 for i = 1:length(x) %216
    for j = 1:length(y)
        f_path(i,j) = vpa(subs(f,[X,Y],[x(i),y(j)]));
         plot3(x(i),y(j),f_path(i,j),"-r",'LineWidth',8);hold on;
    end
 end
%%






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

% [gx, gy] = gradient (-f);
% skip = 2;
% 
% figure;
% 
% xidx = 1:skip:200;
% yidx = 1:skip:17;
% 
% quiver (X(yidx,xidx), Y(yidx,xidx), gx(yidx,xidx), gy(yidx,xidx), 0.4);
% 
% axis ([1 200 1 30]);
% 
% hold on;
% 
% ps = plot(start(1), start(2), 'r.', 'MarkerSize', 30);
% pg = plot(goal(1), goal(2), 'g.', 'MarkerSize', 30);
% p3 = plot (route(:,1), route(:,2), 'r', 'LineWidth', 2);


%%











