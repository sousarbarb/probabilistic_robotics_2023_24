function [e,J]=errorAndJacobianPolarProjectionManifold(X,p,z)
  
  z_hat_cartesian=X(1:3,1:3)*p+X(1:3,4); #point in local frame (cartesian)
  z_hat_polar=c2p(z_hat_cartesian);      #point in polar coordinates
  z_hat_polar=z_hat_polar(2:3);  #eliminate the 1st component (we do not
                                 #observe the range)

  e=z_hat_polar-z;                 #error, we could define it on a
                                   #manifold
                                   #and do the rest of the machinery,
                                   #but we take the short way
                                   #and regard the measurement as
                                   #euclidean
  
  e(1)=atan2(sin(e(1)),cos(e(1))); #of course we need to handle the
                                   #wraparounds of the azimuth

  J_polar=J_c2p(z_hat_cartesian);
  J_polar=J_polar(2:3,:);        #eliminate the 1st component (we do not
                                 #observe the range)
  J=J_polar*J_icp(z_hat_cartesian);
endfunction
