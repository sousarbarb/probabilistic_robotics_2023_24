function [e,J]=errorAndJacobianNormRegistration(X,p,z)
  p_hat=X(1:3,1:3)*p+X(1:3,4); #prediction
  z_hat=p_hat/norm(p_hat);
  e=z_hat-z;
  J_icp=zeros(3,6);
  J_icp(1:3,1:3)=eye(3);
  J_icp(1:3,4:6)=-skew(p_hat);
  J=J_normalize(p_hat)*J_icp;
endfunction
