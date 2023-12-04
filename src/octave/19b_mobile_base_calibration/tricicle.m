1;


function delta_pose=predictTricycle(traction_angular_offset,
                                    measured_steering_angle, x)
  wheel_radius=x(1);
  abs_encoder_offset=x(2);
  baseline = x(3);

  traction_linear_offset   = traction_angular_offset * wheel_radius;
  steering_angle           = measured_steering_angle + abs_encoder_offset;
  back_wheels_displacement = ...
  traction_linear_offset * cos(steering_angle);


  dth = traction_linear_offset * sin(steering_angle) / baseline;
  
  drho = back_wheels_displacement;
  # dx  = drho * (sin(dth)/dth);
  # dy  = drho * (1-cos(dth))/dth;

  S = [0.00000   0.00833   0.00000  -0.16667   0.00000   1.00000];
  C = [0.00139   0.00000  -0.04167   0.00000   0.50000   0.00000];

  dx = drho * polyval(S,dth);
  dy = drho * polyval(C,dth);

  delta_pose=[dx; dy; dth];
  
endfunction


function Z=generateMeasurements(n, x)
  Z=zeros(5,n);
  Z(1:2,:) = rand(2, n);
  for (i = 1:n)
    traction_angular_offset=Z(1,i);
    measured_steering_angle=Z(2,i);
    Z(3:5,i) = predictTricycle(traction_angular_offset,
                               measured_steering_angle,
                               x);
  endfor
endfunction

function [e,J] = errorAndJacobian(x,z)
    traction_angular_offset=z(1);
    measured_steering_angle=z(2);
    meas = z(3:5);

    pred=predictTricycle(traction_angular_offset,
                         measured_steering_angle,
                         x);
    e=pred-meas;
    J=zeros(3,3);
    for (i=1:3)
      epsilon=zeros(3,1);
      epsilon(i)=1e-3;
      J(:,i) = predictTricycle(traction_angular_offset,
                               measured_steering_angle,
                               x+epsilon) ...
               - predictTricycle(traction_angular_offset,
                               measured_steering_angle,
                               x-epsilon);
    endfor;
    J/=2e-3;
endfunction;


function [x, chi]=oneRound(x,Z)
  H=zeros(3,3);
  b=zeros(3,1);
  chi = 0;
  for (i=1:size(Z,2))
    [e,J]=errorAndJacobian(x,Z(:,i));
    chi += e'*e;
    H   += J'*J;
    b   += J'*e;
  endfor
  dx = -H\b;
  x+=dx;
endfunction



