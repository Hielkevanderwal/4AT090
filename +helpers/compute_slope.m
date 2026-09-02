function [obj_slope_coeff, radius_slope_coeff, perpendicular] = compute_slope(obj_name, a, b, x, y, heading)
    % computes slope of the linear function y = m*x
    % INPUTS:
        % obj_name [type: string] = TRACTOR/TRAILER 
        % a, b [type: double] = origin of the circle
        % x, y [type: double] = current pose of tractor/trailer
        % heading [type: double = current heading of tractor/trailer
        
    
    % slope coefficient based on heading of the tractor/trailer
    % measured counterclockwise in radians from the x-axis

    % Exercise 1.4: Slope-based perpendicularity check
    %%%%%%%%%%%% YOUR CODE: START %%%%%%%%%%%%%
    % slope coefficient based on heading of the tractor/trailer
    % measured counterclockwise in radians from the x-axis
    
    obj_slope_coeff = tan(heading);

    % slope coeffinient of the circle radius based on its orignin and current
    % pose of the tractor/trailer

    radius_slope_coeff = (y-b) ./ (x-a);

    % check if the heading of the vehicle is perpentidular to the radius

    perpendicular = radius_slope_coeff * radius_slope_coeff;

    %%%%%%%%%%%%  YOUR CODE: END  %%%%%%%%%%%%%

    
    fprintf(obj_name + "\n slope coeff: " + obj_slope_coeff + "\n turning radius slope coeff: " + radius_slope_coeff + "\n");
    fprintf(obj_name + "\n slope coeff: " + obj_slope_coeff + "\n turning radius slope coeff: " + radius_slope_coeff + "\n");
    fprintf(" m1*m2 = " + perpendicular + "\n");
    
    if abs(perpendicular + 1) < 1e-6
        fprintf(obj_name + " and its respective radious are perpendicular to each other.\n");
    end

end

