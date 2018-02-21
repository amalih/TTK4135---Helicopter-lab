function [c, ceq] = nonlcon(z)
    alfa = 0.2;                                     % constant in ineq. constraint
    beta = 20;                                     % constant in ineq. constraint
    lambda_t = 2*pi/3;
    c = zeros(1,40);
    for i = 1:40
        c(i) = alfa*exp(-beta*(z(1+(6*(i-1))) - lambda_t)^2) - z(5+(6*(i-1)));    % Compute nonlinear inequalities at x.
    end
    ceq = 0;
end 