function [x1,G1,xup,Gup] = kalman_filter(x0,G0,u,y,Galpha,Gbeta,A,C)
   if (isempty(y)),  % When not output (predictor),
       n = length(x0);
       y = eye(0,1);
       Gbeta = eye(0,0);
       C = eye(0,n);
   end
   S = C*G0*C'+Gbeta;
   K = G0*C'/S;
   ytilde = y-C*x0;
   xup = x0+K*ytilde;   % up = update
   Gup = G0-K*C*G0;
   x1 = A*xup+u;
   G1 = A*Gup*A'+Galpha;
end