#this function implements the kalman prediction step of our SLAM system
# inputs:
#   transition: is a data structure containing several things,
#               coming from the simulator
#               of these we are interested only in the offset
#               accessible through the "v" field of the structure
#
#               transition.v(1)=offset x
#               transition.v(2)=offset y (ignore for a diff drive robot)
#               transition.v(3)=offset theta
#
#  considering as 'm' the number of seen landmarks
#  mu: is the mean of (x,y,theta, l1, l2, ..., lm), i.e. the previously estimated robot pose
#      and the m landmark positions
#  sigma: is the covariance of the previously estimated robot pose and landmark positions ((3+m)x(3+m) matrix)

# outputs
# [mu, sigma] are mean and covariance of the estimate after transition

function [mu, sigma] = prediction(mu, sigma, control_input)

  #domain spaces
  dimension_mu = size(mu, 1);
  dimension_u  = 2;

  #get the control input u = [ux, uy, utheta]
  u = control_input.v;

  #readability: current pose
  mu_x     = mu(1);
  mu_y     = mu(2);
  mu_theta = mu(3);

  #readability: current control input
  u_x     = u(1); #translational velocity
  u_theta = u(3); #rotational velocity

  #Jacobian A: df(x, u)/dx
  #initialize A as an identity and fill only the robot block
  A = eye(dimension_mu);
  A(1:3,1:3) = [
    1 , 0 , -u_x * sin(mu_theta) ;
    0 , 1 ,  u_x * cos(mu_theta) ;
    0 , 0 ,  1
  ];

  #Jacobian B: df(x, u)/du
  #for each state variable we have to associate the available control inputs
  B = zeros(dimension_mu,2);
  B(1:3,:) = [
    cos(mu_theta) , 0 ;
    sin(mu_theta) , 0 ;
    0 , 1
  ];

  #predict the robot motion, this is our f(x,u) function in the slides
  #the transition model only affects the robot pose not the landmarks
  mu_r = transition_model(mu(1:3), u);
  #update the robot state
  mu(1:3) = mu_r;

  #control noise u: standard deviations
  sigma_u = 0.1;     #constant part
  sigma_T = u_x;     #translational velocity dependent part
  sigma_R = u_theta; #rotational velocity dependent part

  #compose control noise covariance sigma_u
  sigma_u = diag([ sigma_u ^ 2 + sigma_T ^ 2 , sigma_u ^ 2 + sigma_R ^ 2 ]);

  #predict sigma
  sigma = A * sigma * A' + B * sigma_u * B';
endfunction
