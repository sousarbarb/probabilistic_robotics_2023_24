source '../../tools/utilities/geometry_helpers_3d.m'

# initialize a random set of points
num_points=100;

moving_cloud = 100 * randn(3, num_points);

%projects points on a sphere (no range)
function p_ball = toBall(P)
  p_ball=zeros(2,size(P,2));
  for (i=1:size(P,2))
    p=P(:,i);
    p_ball(:,i)=c2p(p)(2:3);
  endfor;
endfunction;

# ground truth pose that we want to estimate
euclidian_pose = [0.10,-0.5,0.01,pi/4,-pi/4,pi/4]';
ground_truth_pose = v2t(euclidian_pose);

# map the points in the robot frame
fixed_cloud_cartesian = ground_truth_pose(1:3,1:3) * moving_cloud + ground_truth_pose(1:3,4);

#convert to polar, and strip the range
fixed_cloud = toBall(fixed_cloud_cartesian);

# add lil noise
#fixed_cloud+=0.05*(rand(2,num_points)-0.5)

# we don't have any prior knowledge about the uncertanty of individual points
omega = eye(2);

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
    [e,J] = errorAndJacobianPolarProjectionManifold(X,point,measurement);
    # approximate hessian of e'* omega *e
    H += J'*omega*J;
    # gradient of e'* omega * e
    b += J'*omega*e;
    # take track of the chi square
    chi_iteration += e'*omega*e;
  endfor
  # update the current estimate
  chi_square(i) = chi_iteration;
  delta_x = -H\b;
  X = v2t(delta_x) * X;
endfor


t = figure(1)
title("Before optimization")
hold on
plot(fixed_cloud(1,:),fixed_cloud(2,:),'b*','markersize',10);
prediction_start=toBall(moving_cloud);
plot(prediction_start(1,:),prediction_start(2,:),'r.','markersize',10)

t = figure(2)
title("After optimization")
hold on
plot(fixed_cloud(1,:),fixed_cloud(2,:),'b*','markersize',10);
prediction_start=toBall(X(1:3,1:3)*moving_cloud+repmat(X(1:3,4),1,num_points));
plot(prediction_start(1,:),prediction_start(2,:),'r.','markersize',10);

t = figure(3);
title("Squared error evolution")
xlabel("iteration")
ylabel("chi2")
hold on
plot(chi_square,'r','linewidth',2)

X
ground_truth_pose
