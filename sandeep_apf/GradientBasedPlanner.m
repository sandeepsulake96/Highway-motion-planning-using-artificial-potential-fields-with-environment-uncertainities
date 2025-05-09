function route = GradientBasedPlanner (f, start_coords, end_coords, max_its,x,y)
  %function route = GradientBasedPlanner (f, start_coords, end_coords, max_its)
% GradientBasedPlanner : This function plans a path through a 2D
% environment from a start to a destination based on the gradient of the
% function f which is passed in as a 2D array. The two arguments
% start_coords and end_coords denote the coordinates of the start and end
% positions respectively in the array while max_its indicates an upper
% bound on the number of iterations that the system can use before giving
% up.
% The output, route, is an array with 2 columns and n rows where the rows
% correspond to the coordinates of the robot as it moves along the route.
% The first column corresponds to the x coordinate and the second to the y coordinate

[gx, gy] = gradient (-f);

route=start_coords;
point_on_route=start_coords;
speed = 1;
tol = 1;


%%% All of your code should be between the two lines of stars.
% *******************************************************************
while(max_its >0)

    if (norm(end_coords-point_on_route)<tol)
        break;
    end

%     t = (x == point_on_route(1)) & (y == point_on_route(2));
%     indt = find(t);
%     f_grad = [gx(indt) gy(indt)];


    point_Vectorx = ones(size(x))*point_on_route(1);
    [x_closest, x_closest_idx] = min(abs(norm(point_Vectorx-x)));
    
    point_Vectory = ones(size(y))*point_on_route(2);
    [y_closest, y_closest_idx] = min(abs(norm(point_Vectory-y)));

%     flag_grad = (x==x(x_closest_idx)) & (y==point_on_route(2));
%     idx = find(flag_grad);
    %f_grad = [gx(x_closest_idx) gy(y_closest_idx)];


    delta_x = gx(y_closest_idx,x_closest_idx);
    delta_y = gy(y_closest_idx,x_closest_idx);

  

    delta = [delta_x,delta_y];
    %delta = [f_grad(1),f_grad(2)];
    delta_direction_x = delta_x/norm(delta);
    delta_direction_y = delta_y/norm(delta);

    new_route_x = point_on_route(1)-speed*delta_direction_x;
    new_route_y = point_on_route(2)-speed*delta_direction_y;

    point_on_route = [new_route_x,new_route_y];
    route = [route;point_on_route];
    max_its = max_its-1;
end
%
end






