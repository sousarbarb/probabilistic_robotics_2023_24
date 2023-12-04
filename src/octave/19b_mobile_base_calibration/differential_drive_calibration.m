1;


S = [0.00000   0.00833   0.00000  -0.16667   0.00000   1.00000];
C = [0.00139   0.00000  -0.04167   0.00000   0.50000   0.00000];

function v=my_polyval(P,x)
  
endfunction;

function delta = h_odom(x,ticks)
  global S;
  global C;
  kl  = x(1);
  kr  = x(2);
  b   = x(3);
  tl  = ticks(1);
  tr  = ticks(2);
  dl  = tl*kl;
  dr  = tr*kr;
  dth = (dr-dl)/b;
  d_plus2 = 0.5 * (dr+dl);
  dx  =  d_plus2 * polyval(S,dth); # d_plus2 * sin(dth)/dth
  dy  =  d_plus2 * polyval(C,dth); # d_plus2 * (1-cos(dth))/dth
  delta = [dx, dy, dth]';
endfunction

                                #gary kildall

function Z=generateMeasurements(n, x)
                                # zi = tl, tr, dx, dy dth
  
  Z=zeros(5,n);
  Z(1:2,:) = rand(2, n);
  for (i = 1:n)
    ticks = Z(1:2,i);
    Z(3:5,i) = h_odom(x, ticks);
  endfor
endfunction

function [e, J] = errorAndJacobian(x,z)
  ticks = z(1:2);
  meas  = z(3:5);
  pred  = h_odom(x,ticks);
  e     = pred-meas;
  J     = zeros(3,3);
  epsilon = 1e-3;
  inv_eps2= 0.5/epsilon;
  for (i=1:3)
    e_vec = zeros(3);
    e_vec(i)=epsilon;
    J(:,i) = inv_eps2 * (h_odom(x+e_vec, ticks) -h_odom(x-e_vec, ticks));
  endfor;
endfunction

function [x_new, chi] = oneRound(x, Z)
  H=zeros(3,3);
  b=zeros(3,1);
  nmeas=size(Z,2);
  chi=0;
  for (i = 1:nmeas)
    [e,J]=errorAndJacobian(x,Z(:,i));
    H+=J'*J;
    b+=J'*e;
    chi+=e'*e;
  endfor
  dx=-H\b;
  x_new = x+dx;
endfunction
