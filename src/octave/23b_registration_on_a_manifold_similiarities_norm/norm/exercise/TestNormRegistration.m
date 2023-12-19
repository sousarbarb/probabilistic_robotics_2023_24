source '../../../tools/utilities/geometry_helpers_3d.m'

# initialize a random set of points
num_points = 100;
moving_cloud = 100 * randn(3,num_points);

# ground truth pose that we want to estimate
euclidian_pose = [0.10,-0.5,0.01,pi/8,-pi/8,pi/8]';
%euclidian_pose = [0,0,0,0,0,0]';

ground_truth_pose = v2t(euclidian_pose);

%% TODO generate a set of measurements consistent with the error
%% function :)
%% the world sits at euclidean_pose w.r.t. the robot
moving_cloud_hom = ones(4, num_points);
moving_cloud_hom(1:3,:) = moving_cloud;
measurements = ground_truth_pose * moving_cloud_hom;
measurements = measurements(1:3,:);

measurements = measurements ./ sqrt(sum( measurements .^ 2 ))

# we don't have any prior knowledge about the uncertanty of individual points
omega = eye(3);

# initial guess for the pose
X = eye(4);

num_iterations = 10;
chi_square = zeros(num_iterations,1);

%% TODO ICP loop: SPOT the errors
for i=1:num_iterations
  # initialize the quantities of the inner loop
  chi_iteration = 0;
  
  H = zeros(6,6);
  b = zeros(6,1);

  for j=1:num_points

    # compute error and jacobian and update approximate hessian and gradient
    measurement = measurements(:,j);
    point = moving_cloud(:,j);

    [e,J] = errorAndJacobianNormRegistration(X,point,measurement);

    # approximate hessian of e'* omega *e
    H += J'*omega*J;

    # gradient of e'* omega * e
    b += J'*omega*e;

    # take track of the chi square
    chi_iteration += e'*e;

  endfor
  # update the current estimate
  chi_square(i) = chi_iteration;
  delta_x = -H\b;
  rank(H)
  X = v2t(delta_x) * X;
  chi_iteration
endfor

title("Squared error evolution")
xlabel("iteration")
ylabel("chi2")
hold on
plot(chi_square,'r','linewidth',2)
