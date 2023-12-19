source "../../tools/utilities/geometry_helpers_3d.m"

function [e,J]=errorAndJacobian(X,p,z)
	t=X(1:3, 4);  
	R=X(1:3, 1:3);
	s=X(4,4);

	z_hat=s*(R*p+t);
	e=z_hat-z;
  
  J=zeros(3,7);
  J(1:3,1:3)=eye(3);
  J(1:3,4:6)=skew(-z_hat);
  J(1:3,7)=z_hat;
endfunction

function [X, chi_stats, num_inliers]= doICP(x_guess, P, Z, num_iterations, damping, kernel_threshold)
  X=v2s(x_guess);
  chi_stats=zeros(1,num_iterations); 
  num_inliers=zeros(1,num_iterations);
  for (iteration=1:num_iterations)
    H=zeros(7,7);
    b=zeros(7,1);
    chi_stats(iteration)=0;
    for (i=1:size(P,2))
      [e,J] = errorAndJacobian(X, P(:,i), Z(:,i));
      chi = e'*e;
      
      if (chi > kernel_threshold)
	      e*= sqrt(kernel_threshold/chi);
	      chi = kernel_threshold;
      else
	      num_inliers(iteration)++;
      endif;
      
      chi_stats(iteration) += chi; 
      
      H+= J'*J;
      b+= J'*e;
    endfor

    H+= eye(7)*damping;
    dx = -H\b;
    X = v2s(dx)*X;
  endfor
endfunction
