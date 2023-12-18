source '../../tools/utilities/geometry_helpers_3d.m'

%projects points on a sphere (no range)
function p_ball = toBall(P)
  p_ball=zeros(2,size(P,2));
  for (i=1:size(P,2))
    p=P(:,i);
    p_ball(:,i)=c2p(p)(2:3);
  endfor;
endfunction;

# initialize a random set of points in the unit sphere (our stars)
num_points=100;
moving_cloud = randn(3, num_points)-0.5;
for (i = 1:num_points)
  moving_cloud(:,i)=normalize(moving_cloud(:,i));
endfor;

# ground truth pose that we want to estimate
gt_angles = [pi/4,-pi/4,pi/4]';
ground_truth_R = angles2R(gt_angles);

# map the points in the robot frame
fixed_cloud_cartesian = ground_truth_R * moving_cloud;

#convert to polar, and strip the range
fixed_cloud = zeros(2, num_points);
for (i = 1:num_points)
  cp=c2p(fixed_cloud_cartesian(:,i));
  fixed_cloud(:,i)=cp(2:3);
endfor;
# add lil noise
fixed_cloud+=0.05*(rand(2,num_points)-0.5)

omega = eye(2);

# initial guess for the pose
X = eye(3);

num_iterations = 20;
chi_square = zeros(num_iterations,1);

# ICP loop
for i=1:num_iterations
  # initialize the quantities of the inner loop
  chi_iteration = 0;
  H = zeros(3,3);
  b = zeros(3,1);
  for j=1:num_points
    # compute error and jacobian and update approximate hessian and gradient
    measurement = fixed_cloud(:,j);
    point = moving_cloud(:,j);
    [e,J] = errorAndJacobianLookAtStars(X,point,measurement);
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
  X = angles2R(delta_x) * X;
endfor


t = figure(1)
title("Before optimization")
hold on
plot(fixed_cloud(1,:),fixed_cloud(2,:),'b*');
prediction_start=toBall(moving_cloud);
plot(prediction_start(1,:),prediction_start(2,:),'r.')

t = figure(2)
title("After optimization")
hold on
plot(fixed_cloud(1,:),fixed_cloud(2,:),'b*');
prediction_start=toBall(X*moving_cloud);
plot(prediction_start(1,:),prediction_start(2,:),'r.')

t = figure(3)
title("Squared error evolution")
xlabel("iteration")
ylabel("chi2")
hold on
plot(chi_square,'r','linewidth',2)

X
ground_truth_R
