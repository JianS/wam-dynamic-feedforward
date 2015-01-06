
function [ force1, force2 ] = TwoR_ForceManipulability( Cx, Cz, l , tq_limit, originX)
% Calculating the force manipulability of the 2-R robot arm;
% parameters are stated in the parameters.m
% A - base point of robot arm, (xc, yc, zc)
% B - elbow point
% C - end-point, (Cx, Cy)

    r = norm([Cx; Cz]);
    
    if Cx - originX < 0
        phi = -atan(Cz/Cx);
    else
        phi = pi - atan(Cz/Cx);
    end
    
    alpha = acos(r/(2*l));
    
    theta1 = pi/2 + alpha - phi;
    
%     theta2 = asin(r/(2*l)) - phi;
    
    force1 = cross([0;tq_limit;0], [Cx; 0; Cz]/(Cx^2+Cz^2));
    force1 = force1([1 3]); % convert 3-D to 2-D
    
    vec_BC = [Cx; Cz] - [-l*sin(theta1); l*cos(theta1)];
    len_BC = norm(vec_BC);
    force2 = cross( [0;tq_limit;0], [vec_BC(1); 0; vec_BC(2)]/(len_BC^2) );
    force2 = force2([1 3]); % convert 3-D to 2-D

end

