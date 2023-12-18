source '../../tools/utilities/geometry_helpers_3d.m'

# initialize a random set of points
moving_cloud = 100 * randn(3,1000);

# ground truth pose that we want to estimate
euclidian_pose = [0.10,-0.5,0.01,pi/4,-pi/16,pi]';
ground_truth_pose = v2t(euclidian_pose);

# generate measurements
fixed_cloud = ground_truth_pose(1:3,1:3) * moving_cloud + ground_truth_pose(1:3,4);

# we don't have any prior knowledge about the uncertanty of individual points
omega = eye(3);

num_points = size(fixed_cloud,1);

# initial guess for the pose
X = eye(4);

num_iterations = 20;
chi_square = zeros(num_iterations,1);

# ICP loop
for i=1:num_iterations
  # initialize the quantities of the inner loop
  chi_iteration = 0;
  H = zeros(6,6);
  b = zeros(6,1);
  for j=1:num_points
    # compute error and jacobian and update approximate hessian and gradient
    measurement = fixed_cloud(:,j);
    point = moving_cloud(:,j);
    [e,J] = errorAndJacobianManifold(X,point,measurement);
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
  X = v2t(delta_x) * X;
endfor

t = figure(1)
title("Before optimization")
hold on
plot3(moving_cloud(1,:),moving_cloud(2,:),moving_cloud(3,:),'b*')
plot3(fixed_cloud(1,:),fixed_cloud(2,:),fixed_cloud(3,:),'r*')
t = figure(2)
title("After optimization")
hold on
estimated_cloud = X(1:3,1:3) * moving_cloud + X(1:3,4);
plot3(estimated_cloud(1,:),estimated_cloud(2,:),estimated_cloud(3,:),'b*')
plot3(fixed_cloud(1,:),fixed_cloud(2,:),fixed_cloud(3,:),'r*')
t = figure(3)
title("Squared error evolution")
xlabel("iteration")
ylabel("chi2")
hold on
plot(chi_square,'r','linewidth',2)
