
function [ theta1, theta2 ] = TwoR_InvKin( Cx, Cz, l , originX)
% Calculating the inverse kinematic of the 2-R robot arm;
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
    
    theta2 = asin(r/(2*l)) - phi;

    
end

