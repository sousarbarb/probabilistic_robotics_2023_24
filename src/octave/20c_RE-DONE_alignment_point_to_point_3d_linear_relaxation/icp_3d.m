source "../tools/utilities/geometry_helpers_3d.m"

function [e,J]=errorAndJacobian(x,p,z)

  M = zeros(3,9);
  M(1, 1:3) = p(1:3);
  M(2, 4:6) = p(1:3);
  M(3, 7:9) = p(1:3);
  
  h = M * x(1:9,1) + x(10:12);
  e = h - z;
  
  J = zeros(3,12);
  J(1:3,1:9) = M;
  J(1:3, 10:12) = eye(3);

endfunction

function [R, t]=doICP(x_guess, P, Z)
  
  H = zeros(12,12);
  b = zeros(12,1);

  for (i=1:size(P,2))
    [e,J] = errorAndJacobian(x_guess', P(:,i), Z(:,i));
    H+=J'*J;
    b+=J'*e;
  endfor

  dx=-H\b;
  x_guess+=dx';

  
  R = [x_guess(1:3);
       x_guess(4:6);
       x_guess(7:9)];
  
  [U, s, V] = svd(R);
  
  R = U * V';
  t = x_guess(1, 10:12);
  
endfunction
