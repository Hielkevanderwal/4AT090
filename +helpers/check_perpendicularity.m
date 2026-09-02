function check_perpendicularity(obj_name,a,b,x,y,heading)

    % Exercise 1.5: Perpendicularity check
    %%%%%%%%%%%% YOUR CODE: START %%%%%%%%%%%%%
    v = [cos(heading); sin(heading)];

    u = [x - a; y- b];
    
    dotProduct = dot(u, v);


    %%%%%%%%%%%%  YOUR CODE: END  %%%%%%%%%%%%%

    if abs(dotProduct) < 1e-6
        fprintf(obj_name + ' heading and its turning radius are perpendicular.\n');
    else
        fprintf(obj_name + ' heading and its turning radius are NOT perpendicular.\n');
    end
end