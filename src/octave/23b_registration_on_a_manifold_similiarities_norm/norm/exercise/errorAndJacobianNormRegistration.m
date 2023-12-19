function [e,J]=errorAndJacobianNormRegistration(X,p,z)

  z_hat = normalize(X(1:3,1:3) * p + X(1:3,4));
  e = z_hat - z;

  J = zeros(3,6);
  J(1:3,1:3) = eye(3);
  J(1:3,4:6) = -skew(z_hat);

  J = (eye(3) * (norm(z_hat).^2) - z_hat * z_hat') * J ./ (norm(z_hat).^3);

endfunction
