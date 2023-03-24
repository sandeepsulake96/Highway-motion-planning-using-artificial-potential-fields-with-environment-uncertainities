
function [route]= grad_desc(start,goal,f,max_iter,dist_tol) 
syms X Y;
route=start;
point_on_route=start;
x(1) = start(1);
y(1) = start(2);
i=1;
% Gradient Computation:
df_dx = diff(f, X);
df_dy = diff(f, Y);
J = [subs(df_dx,[X,Y], [x(1),y(1)]) subs(df_dy, [X,Y], [x(1),y(1)])]; % Gradient
S = -(J); % Search Direction
hx=0.5;     %1.5
hy=0.01;   %0.01
%e=err_tol;

%while norm(goal-point_on_route)> dist_tol
while(max_iter >0)

    %I = [x(i),y(i)];
    
    if (norm(goal-point_on_route)< dist_tol)
        break;
    end
    
%     syms h; % Step size
%     g = subs(f, [X,Y], [x(i)+S(1)*h,y(i)+h*S(2)]);
%     dg_dh = diff(g,h);
%     h = solve(dg_dh, h); % Optimal Step Length
    
    x(i+1) = point_on_route(1)+hx*S(1);
    y(i+1) = point_on_route(2)+hy*S(2);
    
    point_on_route = [x(i+1),y(i+1)];
    route = [route;x(i+1),y(i+1)];
    max_iter = max_iter-1;

    %x(i+1) = I(1)+h*S(1); % Updated x value
    %y(i+1) = I(2)+0.01*S(2); % Updated y value
    i = i+1;
    J = [subs(df_dx,[X,Y], [x(i),y(i)]) subs(df_dy, [X,Y], [x(i),y(i)])]; % Updated Gradient
    S = -(J); % New Search Direction
end
end
