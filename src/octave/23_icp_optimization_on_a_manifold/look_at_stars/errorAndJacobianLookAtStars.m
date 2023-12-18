function [e,J]=errorAndJacobianLookAtStars(X,p,z)
  
  z_hat_cartesian=X*p;    #point in local frame (only the
                          #rotation applies)
  
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
  J=-J_polar*skew(z_hat_cartesian);
endfunction
